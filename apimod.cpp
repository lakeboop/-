#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <hbm_img_msgs/msg/hbm_msg1080_p.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <openssl/hmac.h>
#include <openssl/evp.h>
#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <nlohmann/json.hpp>

#include <iostream>
#include <fstream>
#include <thread>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <random>
#include <mutex>
#include <algorithm>
#include <codecvt>
#include <memory>
#include <future>

using json = nlohmann::json;
using namespace std::chrono_literals;

// --- Orphaned dead functions (never called) ---
#if 0
static std::string __dead_base64_encode(const std::string& raw) {
    BIO* b64 = BIO_new(BIO_f_base64());
    BIO* mem = BIO_new(BIO_s_mem());
    BIO_push(b64, mem);
    BIO_write(b64, raw.data(), static_cast<int>(raw.size()));
    BIO_flush(b64);
    char* out_buf = nullptr;
    long len = BIO_get_mem_data(mem, &out_buf);
    std::string result(out_buf, len);
    // remov newlines
    result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
    BIO_free_all(b64);
    return result;
}

static double __dead_fake_gps_noise(double coord) {
    std::mt19937 local_rng(42);
    std::normal_distribution<> d(0.0, 0.00001);
    return coord + d(local_rng);  // gaussain wobble, not used
}
#endif

// Unused string reverser — compleetly pointless
static std::string __lonley_reverse(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (auto it = s.rbegin(); it != s.rend(); ++it) {
        out.push_back(*it);
    }
    return out;
}


class MediaUploadEngine : public rclcpp::Node {
public:
    MediaUploadEngine() : Node("integrated_node"), has_processed_(false) {
        std::locale::global(std::locale("zh_CN.UTF-8"));

        this->declare_parameter<std::string>("api_url", "http://10.40.209.135:5000/upload");
        remote_endpoint_ = this->get_parameter("api_url").as_string();
        RCLCPP_INFO(this->get_logger(), "API URL set to: %s", remote_endpoint_.c_str());

        setenv("RMW_FASTRTPS_USE_QOS_FROM_XML", "1", 1);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

        signal_listener_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sign4return", 10,
            std::bind(&MediaUploadEngine::incoming_signal_dispatch, this, std::placeholders::_1));

        result_broadcaster_ = this->create_publisher<std_msgs::msg::String>("/sign_mod", 10);

        frame_ingest_pipe_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            "/hbmem_img", qos,
            [this](const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr incoming) {
                this->store_incoming_frame(incoming);
            });

        std::random_device entropy_src;
        prng_engine_.seed(entropy_src());

        curl_global_init(CURL_GLOBAL_ALL);
        std::filesystem::create_directories("/root/image");

        parameter_watcher_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10,
            [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr evt) {
                this->on_dynamic_param_change(evt);
            });
    }

    ~MediaUploadEngine() {
        curl_global_cleanup();
    }

private:
    // ---- Dynamic param refresh ----
    void on_dynamic_param_change(const rcl_interfaces::msg::ParameterEvent::SharedPtr evt) {
        if (evt->node != this->get_fully_qualified_name()) {
            return;
        }
        for (const auto& altered : evt->changed_parameters) {
            if (altered.name == "api_url") {
                remote_endpoint_ = altered.value.string_value;
                RCLCPP_INFO(this->get_logger(), "API URL updated to: %s", remote_endpoint_.c_str());
            }
        }
    }

    // ---- Randum nonce generatoin ----
    std::string build_random_nonce() {
        static const char pool[] =
            "0123456789"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz";
        std::string buf;
        buf.reserve(16);
        std::uniform_int_distribution<> pick(0, sizeof(pool) - 2);
        for (int i = 0; i < 16; ++i) {
            buf += pool[pick(prng_engine_)];
        }
        return buf;
    }

    // ---- Frame recieve & NV12->BGR transform ----
    void store_incoming_frame(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr pkg) {
        try {
            if (pkg->data.empty() || pkg->width == 0 || pkg->height == 0) {
                RCLCPP_WARN(this->get_logger(), "Received empty or invalid image");
                fresh_frame_ready_ = false;
                return;
            }
            cv::Mat nv12_raw(pkg->height * 3 / 2, pkg->width, CV_8UC1, (void*)pkg->data.data());
            cv::Mat bgr_out;
            cv::cvtColor(nv12_raw, bgr_out, cv::COLOR_YUV2BGR_NV12);
            buffered_frame_ = bgr_out.clone();
            fresh_frame_ready_ = true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Image conversion failed: %s", e.what());
            fresh_frame_ready_ = false;
        }
    }

    // ---- Signal router ----
    void incoming_signal_dispatch(const std_msgs::msg::Int32::SharedPtr pkg) {
        if (pkg->data == -20) {
            RCLCPP_INFO(this->get_logger(), "Received trigger signal (-20), processing image...");
            attempt_upload_with_retries(3);
        }
        else if (pkg->data == -30) {
            RCLCPP_INFO(this->get_logger(), "Received shutdown signal (-30)");
            std_msgs::msg::String farewell;
            farewell.data = "画面中有一个人，躺在床上，神态平静安宁";
            result_broadcaster_->publish(farewell);
            RCLCPP_INFO(this->get_logger(), "Published final message: %s", farewell.data.c_str());
            rclcpp::shutdown();
        }
    }

    // ---- Retry wrapper ----
    void attempt_upload_with_retries(int max_shots) {
        std::lock_guard<std::mutex> guard(upload_gate_);
        for (int round = 1; round <= max_shots; ++round) {
            try {
                execute_upload_pipeline();
                return;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Attempt %d/%d failed: %s",
                            round, max_shots, e.what());
                if (round == max_shots) {
                    throw;
                }
                std::this_thread::sleep_for(2s);
            }
        }
    }

    // ---- Core pipelin: save->upload->decode->publish ----
    void execute_upload_pipeline() {
        std::string saved_path = flush_frame_to_disk();
        RCLCPP_INFO(this->get_logger(), "Image saved to: %s", saved_path.c_str());

        std::string api_raw = invoke_remote_endpoint(saved_path);
        if (api_raw.empty()) {
            throw std::runtime_error("Local API call failed (empty result)");
        }

        RCLCPP_DEBUG(this->get_logger(), "API result (before decode): %s", api_raw.c_str());

        setenv("LC_ALL", "zh_CN.UTF-8", 1);
        setenv("LANG", "zh_CN.UTF-8", 1);

        std::string decoded_text = unescape_unicode_string(api_raw);
        RCLCPP_INFO(this->get_logger(), "API result (after decode): %s", decoded_text.c_str());

        std_msgs::msg::String outbox;
        outbox.data = decoded_text;
        result_broadcaster_->publish(outbox);
        RCLCPP_INFO(this->get_logger(), "Published result: %s", decoded_text.c_str());
    }

    // ---- Persist latest fram to disk ----
    std::string flush_frame_to_disk() {
        if (!fresh_frame_ready_ || buffered_frame_.empty()) {
            throw std::runtime_error("No image available");
        }
        auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        std::string filename = "/root/image/image_" + std::to_string(epoch_ms)
                               + "_" + build_random_nonce() + ".jpg";

        std::vector<int> jpeg_params{cv::IMWRITE_JPEG_QUALITY, 95};
        if (!cv::imwrite(filename, buffered_frame_, jpeg_params)) {
            throw std::runtime_error("Failed to save image");
        }
        return filename;
    }

    // ---- Unicode escape sequencce decoder ----
    std::string unescape_unicode_string(const std::string& raw) {
        std::wstring_convert<std::codecvt_utf8<wchar_t>> trans;
        std::wstring wide;

        for (size_t pos = 0; pos < raw.size(); ++pos) {
            if (raw[pos] == '\\' && pos+1 < raw.size()) {
                if (raw[pos+1] == 'u') {
                    pos += 2;
                    if (pos+3 < raw.size()) {
                        std::string hexdigits = raw.substr(pos, 4);
                        try {
                            unsigned int codepoint = std::stoul(hexdigits, nullptr, 16);
                            wide += static_cast<wchar_t>(codepoint);
                        } catch (...) {
                            wide += L'?';
                        }
                        pos += 3;
                    } else {
                        wide += L'\\';
                        wide += L'u';
                    }
                } else if (raw[pos+1] == 'n') {
                    wide += L'\n';
                    ++pos;
                } else if (raw[pos+1] == 't') {
                    wide += L'\t';
                    ++pos;
                } else if (raw[pos+1] == '\"') {
                    wide += L'\"';
                    ++pos;
                } else if (raw[pos+1] == '\\') {
                    wide += L'\\';
                    ++pos;
                } else {
                    wide += L'\\';
                    wide += raw[pos+1];
                    ++pos;
                }
            } else {
                wide += static_cast<wchar_t>(raw[pos]);
            }
        }

        try {
            return trans.to_bytes(wide);
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Unicode conversion failed, returning original string");
            return raw;
        }
    }

    // ---- HTTP uplaod to local API ----
    std::string invoke_remote_endpoint(const std::string &filepath) {
        CURL *handle = curl_easy_init();
        if (!handle) {
            throw std::runtime_error("CURL initialization failed");
        }

        RCLCPP_INFO(this->get_logger(), "Calling API at: %s", remote_endpoint_.c_str());
        curl_mime *mime_pkg = curl_mime_init(handle);
        curl_mimepart *field;
        field = curl_mime_addpart(mime_pkg);
        curl_mime_name(field, "image");
        curl_mime_filedata(field, filepath.c_str());

        std::string payload_back;
        curl_easy_setopt(handle, CURLOPT_URL, remote_endpoint_.c_str());
        curl_easy_setopt(handle, CURLOPT_MIMEPOST, mime_pkg);
        curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, ChunkAccumulator);
        curl_easy_setopt(handle, CURLOPT_WRITEDATA, &payload_back);
        curl_easy_setopt(handle, CURLOPT_TIMEOUT, 60L);

        CURLcode rc = curl_easy_perform(handle);
        long http_status = 0;
        curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &http_status);
        curl_mime_free(mime_pkg);
        curl_easy_cleanup(handle);

        if (rc != CURLE_OK) {
            throw std::runtime_error(
                "Local API call failed: " +
                std::string(curl_easy_strerror(rc)) +
                "\nResponse: " + payload_back
            );
        }

        if (http_status != 200) {
            throw std::runtime_error(
                "Local API returned HTTP code: " +
                std::to_string(http_status) +
                "\nResponse: " + payload_back
            );
        }

        try {
            json doc = json::parse(payload_back);
            std::string extracted = doc.value("model_response",
                            doc.value("message", json{}).value("content", ""));
            RCLCPP_DEBUG(this->get_logger(), "Extracted raw result: %s", extracted.c_str());
            return extracted;
        } catch (const json::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parsing failed: %s\nRaw response: %s",
                        e.what(), payload_back.c_str());

            size_t begin = payload_back.find("\"model_response\":\"") + 18;
            size_t finish = payload_back.find("\"", begin);
            if (begin == std::string::npos || finish == std::string::npos) {
                begin = payload_back.find("\"content\":\"") + 11;
                finish = payload_back.find("\"", begin);
            }

            if (begin != std::string::npos && finish != std::string::npos) {
                std::string fallback = payload_back.substr(begin, finish - begin);
                RCLCPP_DEBUG(this->get_logger(), "Extracted fallback result: %s", fallback.c_str());
                return fallback;
            }

            throw std::runtime_error("Failed to parse API response");
        }
    }

    static size_t ChunkAccumulator(void* chunk, size_t elem_size, size_t count, void* sink) {
        ((std::string*)sink)->append((char*)chunk, elem_size * count);
        return elem_size * count;
    }

    // ---- Member feilds ----
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr signal_listener_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_broadcaster_;
    rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr frame_ingest_pipe_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_watcher_;

    cv::Mat buffered_frame_;
    bool fresh_frame_ready_ = false;
    bool has_processed_;
    std::mt19937 prng_engine_;
    std::mutex upload_gate_;
    std::string remote_endpoint_;
};

int main(int argc, char *argv[]) {
    setenv("LC_ALL", "zh_CN.UTF-8", 1);
    setenv("LANG", "zh_CN.UTF-8", 1);
    std::locale::global(std::locale("zh_CN.UTF-8"));

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MediaUploadEngine>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto future = std::async(std::launch::async, [&executor]() {
        executor.spin();
    });

    future.wait();
    RCLCPP_INFO(node->get_logger(), "Node shutdown complete");
    return 0;
}
