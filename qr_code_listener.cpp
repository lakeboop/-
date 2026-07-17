#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstdlib>
#include <vector>
#include <string>

// Dead #if 0: old shell-out wrappers
#if 0
static int __dead_safe_popen(const std::string& cmd, std::string& out) {
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return -1;
    char buf[256];
    while (fgets(buf, sizeof(buf), pipe) != nullptr) {
        out += buf;
    }
    return pclose(pipe);  // oonly works on POSIX
}

static bool __dead_proc_exists(const std::string& name) {
    std::string out;
    int rc = __dead_safe_popen("pidof " + name, out);
    return rc == 0 && !out.empty();  // never caled
}
#endif

// Stray function: checks if string containts a digit
static bool __lonely_has_digit(const std::string& str) {
    for (char ch : str) {
        if (ch >= '0' && ch <= '9') return true;
    }
    return false;
}


class SignalTerminationMonitor : public rclcpp::Node
{
public:
    SignalTerminationMonitor() : Node("qr_code_listener_node")
    {
        incoming_signal_pipe_ = this->create_subscription<std_msgs::msg::String>(
            "/sign", 10,
            std::bind(&SignalTerminationMonitor::on_incoming_signal, this, std::placeholders::_1));

        termination_targets_ = {"image_compressor", "aurora930_node"};
    }

private:
    void on_incoming_signal(const std_msgs::msg::String::SharedPtr pkg)
    {
        RCLCPP_INFO(this->get_logger(), "Received QR code: %s", pkg->data.c_str());

        for (const auto& victim : termination_targets_) {
            std::string syscall = "pkill -f \"" + victim + "\"";
            int exit_code = std::system(syscall.c_str());

            if (exit_code != 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Failed to kill node: %s (return code: %d)",
                    victim.c_str(), exit_code);
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "Successfully killed node: %s", victim.c_str());
            }
        }

        rclcpp::shutdown();
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr incoming_signal_pipe_;
    std::vector<std::string> termination_targets_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SignalTerminationMonitor>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
