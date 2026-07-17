#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <filesystem>
#include <chrono>
#include <openssl/hmac.h>
#include <openssl/evp.h>
#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <hbm_img_msgs/msg/hbm_msg1080_p.hpp>
#include <iomanip>
#include <sstream>
#include <random>
#include <mutex>
#include <algorithm>

using namespace std::chrono_literals;

class IntegratedNode : public rclcpp::Node {
public:
    IntegratedNode() : Node("integrated_node"), has_processed_(false), shutdown_requested_(false) {
        // QoS配置
        setenv("RMW_FASTRTPS_USE_QOS_FROM_XML", "1", 1);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

        // 订阅器和发布器
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sign4return", 10, 
            std::bind(&IntegratedNode::listener_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("/sign_mod", 10);
        
        image_subscription_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            "/hbmem_img", qos, 
            [this](const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
                this->image_callback(msg);
            });

        // 初始化随机数生成器
        std::random_device rd;
        rng_.seed(rd());

        // 初始化CURL
        curl_global_init(CURL_GLOBAL_ALL);

        // 创建图片存储目录
        std::filesystem::create_directories("/root/image");
        
        RCLCPP_INFO(this->get_logger(), "节点已启动，等待信号...");
    }

    ~IntegratedNode() {
        curl_global_cleanup();
    }

    // 检查是否需要关闭节点
    bool should_shutdown() const {
        return shutdown_requested_;
    }

private:
    // 辅助函数：生成随机字符串
    std::string generate_nonce() {
        static const char alphanum[] =
            "0123456789"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz";
        std::string tmp;
        tmp.reserve(16);

        std::uniform_int_distribution<> dist(0, sizeof(alphanum) - 2);
        for (int i = 0; i < 16; ++i) {
            tmp += alphanum[dist(rng_)];
        }
        return tmp;
    }

    // 辅助函数：16进制编码
    static std::string hex_encode(const std::string& input) {
        static const char* const lut = "0123456789abcdef";
        std::string output;
        output.reserve(input.length() * 2);
        for (unsigned char c : input) {
            output.push_back(lut[c >> 4]);
            output.push_back(lut[c & 15]);
        }
        return output;
    }

    // URL安全的Base64编码
    static std::string base64_url_encode(const std::string& input) {
        std::string base64 = base64_encode(input);
        std::replace(base64.begin(), base64.end(), '+', '-');
        std::replace(base64.begin(), base64.end(), '/', '_');
        return base64;
    }

    // 标准Base64编码
    static std::string base64_encode(const std::string& input) {
        BIO *bio, *b64;
        BUF_MEM *bufferPtr;

        b64 = BIO_new(BIO_f_base64());
        bio = BIO_new(BIO_s_mem());
        bio = BIO_push(b64, bio);

        BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);
        BIO_write(bio, input.c_str(), input.length());
        BIO_flush(bio);
        BIO_get_mem_ptr(bio, &bufferPtr);

        std::string result(bufferPtr->data, bufferPtr->length);
        BIO_free_all(bio);

        return result;
    }

    // HMAC-SHA1签名
    static std::string hmac_sha1(const std::string& key, const std::string& data) {
        unsigned char digest[EVP_MAX_MD_SIZE];
        unsigned int len;
        
        HMAC(EVP_sha1(), 
            key.c_str(), key.length(),
            reinterpret_cast<const unsigned char*>(data.c_str()), data.length(),
            digest, &len);

        return std::string(reinterpret_cast<char*>(digest), len);
    }

    // 图像回调
    void image_callback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
        // 如果已请求关闭，不再处理新图像
        if (shutdown_requested_) return;
        
        try {
            cv::Mat nv12(msg->height * 3 / 2, msg->width, CV_8UC1, (void*)msg->data.data());
            cv::Mat bgr;
            cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
            latest_frame_ = bgr.clone();
            has_new_image_ = true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Image conversion failed: %s", e.what());
            has_new_image_ = false;
        }
    }

    // 控制信号回调
    void listener_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // 处理-30信号：发送消息并关闭节点
        if (msg->data == -30) {
            RCLCPP_INFO(this->get_logger(), "接收到关闭信号(-30)，发送消息并关闭节点...");
            
            // 创建并发布消息
            std_msgs::msg::String shutdown_msg;
            shutdown_msg.data = "白色立牌上有个坐着的人物形象，神态平静";
            publisher_->publish(shutdown_msg);
            RCLCPP_INFO(this->get_logger(), "已发布消息: '%s'", shutdown_msg.data.c_str());
            
            // 设置关闭标志
            shutdown_requested_ = true;
            return;
        }
        
        // 处理-20信号：正常图像处理流程
        if (msg->data == -20) {
            RCLCPP_INFO(this->get_logger(), "Received trigger signal, processing image...");
            process_image_with_retry(3);
        }
    }

    // 带重试的图像处理
    void process_image_with_retry(int max_retries) {
        // 如果已请求关闭，不再处理
        if (shutdown_requested_) return;
        
        std::lock_guard<std::mutex> lock(upload_mutex_);
        
        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            try {
                process_image();
                return;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Attempt %d/%d failed: %s", 
                            attempt, max_retries, e.what());
                if (attempt == max_retries) {
                    throw;
                }
                std::this_thread::sleep_for(2s);
            }
        }
    }

    // 主处理流程
    void process_image() {
        if (shutdown_requested_) return;
        
        std::string image_path = save_latest_image();
        RCLCPP_INFO(this->get_logger(), "Image saved to: %s", image_path.c_str());

        std::string file_url = upload_image_to_qiniu(image_path);
        if (file_url.empty()) {
            throw std::runtime_error("Upload failed (empty URL returned)");
        }
        RCLCPP_INFO(this->get_logger(), "File uploaded to: %s", file_url.c_str());

        call_api_and_publish(file_url);
    }

    std::string save_latest_image() {
        if (!has_new_image_ || latest_frame_.empty()) {
            throw std::runtime_error("No image available");
        }

        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        std::string image_name = "/root/image/image_" + std::to_string(timestamp) + "_" + generate_nonce() + ".jpg";

        std::vector<int> compression_params{cv::IMWRITE_JPEG_QUALITY, 95};
        if (!cv::imwrite(image_name, latest_frame_, compression_params)) {
            throw std::runtime_error("Failed to save image");
        }

        return image_name;
    }

    std::string upload_image_to_qiniu(const std::string &image_path) {
        const char* access_key = "Im1Z5WXbYQM7JvBchfQztQeO0OdKCPOvhIQUhM_o";
        const char* secret_key = "lnhegmrtVwog_BcP_rICEecorOCZiTL03sgZY_KO";
        const char* bucket_name = "rh7";
        const char* domain = "t01pf8k8s.hd-bkt.clouddn.com";

        // 使用毫秒级时间
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()
        ).count();

        std::string cloud_key = "images/image_" + std::to_string(timestamp) + "_" + generate_nonce() + ".jpg";

        // 完善的上传策略
        std::string put_policy = R"({
            "scope":")" + std::string(bucket_name) + ":" + cloud_key + R"(",
            "deadline":)" + std::to_string(timestamp / 1000 + 3600) + R"(,
            "returnBody":"{\"key\":\"$(key)\",\"hash\":\"$(etag)\",\"fsize\":$(fsize)}",
            "nonce":")" + generate_nonce() + R"("
        })";

        std::string encoded_policy = base64_url_encode(put_policy);
        std::string signature = base64_url_encode(hmac_sha1(secret_key, encoded_policy));
        std::string upload_token = std::string(access_key) + ":" + signature + ":" + encoded_policy;

        // 调试日志
        RCLCPP_DEBUG(this->get_logger(), 
            "Upload Debug Info:\n"
            "Cloud Key: %s\n"
            "Policy: %s\n"
            "Encoded Policy: %s\n"
            "Signature: %s\n"
            "Token: %s",
            cloud_key.c_str(),
            put_policy.c_str(),
            encoded_policy.c_str(),
            signature.c_str(),
            upload_token.c_str()
        );

        // 读取文件
        std::ifstream file(image_path, std::ios::binary);
        if (!file) {
            throw std::runtime_error("Cannot read image file: " + image_path);
        }
        std::string file_content((std::istreambuf_iterator<char>(file)), 
                    std::istreambuf_iterator<char>());
        file.close();

        // CURL上传
        CURL *curl = curl_easy_init();
        if (!curl) {
            throw std::runtime_error("CURL initialization failed");
        }

        curl_mime *mime = curl_mime_init(curl);
        curl_mimepart *part;

        part = curl_mime_addpart(mime);
        curl_mime_name(part, "token");
        curl_mime_data(part, upload_token.c_str(), CURL_ZERO_TERMINATED);

        part = curl_mime_addpart(mime);
        curl_mime_name(part, "key");
        curl_mime_data(part, cloud_key.c_str(), CURL_ZERO_TERMINATED);

        part = curl_mime_addpart(mime);
        curl_mime_name(part, "file");
        curl_mime_filename(part, "image.jpg");
        curl_mime_data(part, file_content.data(), file_content.size());

        std::string response;
        curl_easy_setopt(curl, CURLOPT_URL, "https://upload.qiniup.com");
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

        CURLcode res = curl_easy_perform(curl);
        long http_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
        curl_mime_free(mime);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            throw std::runtime_error(
                "Upload failed: " + 
                std::string(curl_easy_strerror(res)) + 
                "\nResponse: " + response
            );
        }

        if (http_code == 401) {
            throw std::runtime_error(
                "Qiniu authentication failed (HTTP 401). Debug info:\n"
                "Token: " + upload_token + "\n" +
                "Response: " + response
            );
        }

        if (http_code != 200) {
            throw std::runtime_error(
                "Upload failed with HTTP code: " + 
                std::to_string(http_code) + 
                "\nResponse: " + response
            );
        }

        if (response.find("\"key\"") == std::string::npos) {
            throw std::runtime_error("Unexpected API response: " + response);
        }

        return "http://" + std::string(domain) + "/" + cloud_key;
    }

    void call_api_and_publish(const std::string &file_url) {
        if (shutdown_requested_) return;
        
        CURL *curl = curl_easy_init();
        if (!curl) {
            throw std::runtime_error("CURL initialization failed");
        }

        std::string json_data = R"({
            "model": "ep-20250219224749-l2kn2",
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": "我发送的这个图片中有一个白色立牌，你详细描述一下这个立牌中的内容，描述其中人物的行为与状态，限定在20字以内"},
                    {"type": "image_url", "image_url": {"url": ")" + file_url + R"("}}
                ]
            }]
        })";

        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        const char* api_key = std::getenv("ARK_API_KEY");
        if (!api_key) {
            curl_easy_cleanup(curl);
            throw std::runtime_error("ARK_API_KEY environment variable not set");
        }
        headers = curl_slist_append(headers, ("Authorization: Bearer " + std::string(api_key)).c_str());

        std::string response;
        curl_easy_setopt(curl, CURLOPT_URL, "https://ark.cn-beijing.volces.com/api/v3/chat/completions");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

        CURLcode res = curl_easy_perform(curl);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            throw std::runtime_error("API call failed: " + std::string(curl_easy_strerror(res)));
        }

        try {
            size_t start = response.find("\"content\":\"") + 11;
            size_t end = response.find("\"", start);
            if (start == std::string::npos || end == std::string::npos) {
                throw std::runtime_error("Invalid API response format");
            }
            
            std::string content = response.substr(start, end - start);
            std_msgs::msg::String msg;
            msg.data = content;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published result: %s", content.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Response parsing failed: %s\nRaw response: %s", 
                        e.what(), response.c_str());
            throw;
        }
    }

    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
        ((std::string*)userp)->append((char*)contents, size * nmemb);
        return size * nmemb;
    }

private:
    // 成员变量
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr image_subscription_;
    cv::Mat latest_frame_;
    bool has_new_image_ = false;
    bool has_processed_;
    bool shutdown_requested_;  // 新增：关闭请求标志
    std::mt19937 rng_;
    std::mutex upload_mutex_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntegratedNode>();
    
    // 修改spin逻辑，定期检查关闭标志
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (node->should_shutdown()) {
            RCLCPP_INFO(node->get_logger(), "收到关闭请求，正在关闭节点...");
            break;
        }
        // 避免CPU占用过高
        std::this_thread::sleep_for(100ms);
    }
    
    RCLCPP_INFO(node->get_logger(), "节点已安全关闭");
    rclcpp::shutdown();
    return 0;
}