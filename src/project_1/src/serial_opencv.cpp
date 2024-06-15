#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "socket_communication/msg/opencv1.hpp"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <libserialport.h>
#include <string>

using namespace std::chrono_literals;

sp_return ret;

class OpencvNode : public rclcpp::Node {
public:
    OpencvNode() : Node("opencv_node") {
        publisher_ = this->create_publisher<socket_communication::msg::Opencv1>("opencv_topic", 10);

        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0) {
            perror("Error creating socket");
            return;
        }

        server_address_.sin_family = AF_INET;
        server_address_.sin_addr.s_addr = INADDR_ANY;
        server_address_.sin_port = htons(12345);

        if (bind(server_socket_, (struct sockaddr *)&server_address_, sizeof(server_address_)) < 0) {
            perror("Error binding socket");
            return;
        }

        listen(server_socket_, 1);
        std::cout << "Socket server listening..." << std::endl;

        client_socket_ = accept(server_socket_, NULL, NULL);
        if (client_socket_ < 0) {
            perror("Error accepting connection");
            return;
        }

        std::thread(&OpencvNode::publishMessages, this).detach();
    }

private:
    void publishMessages() {
        while (rclcpp::ok()) {
            char buffer[1024] = {0};
            int valread = read(client_socket_, buffer, 1024);
            if (valread <= 0) {
                break;
            }

            int data_1, data_2;
            sscanf(buffer, "%d;%d", &data_1, &data_2);

            auto msg = socket_communication::msg::Opencv1();
            msg.data_flag = data_1;
            msg.flag_condition = data_2;

            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: data_1=%d, data_2=%d", msg.data_flag, msg.flag_condition);
        }
    }

    int server_socket_;
    int client_socket_;
    struct sockaddr_in server_address_;
    rclcpp::Publisher<socket_communication::msg::Opencv1>::SharedPtr publisher_;
};

class SerialNode : public rclcpp::Node {
public:
    SerialNode() : Node("serial_node"), serial_port_(nullptr) {
        subscription_ = this->create_subscription<socket_communication::msg::Opencv1>(
            "opencv_topic", 10, std::bind(&SerialNode::socketTopicCallback, this, std::placeholders::_1));

        sp_get_port_by_name("/dev/serial/by-path/pci-0000:06:00.4-usb-0:2:1.0-port0", &serial_port_);
        sp_set_baudrate(serial_port_, 9600);
        sp_set_bits(serial_port_, 8);
        sp_set_parity(serial_port_, SP_PARITY_NONE);
        sp_set_stopbits(serial_port_, 1);
        sp_close(serial_port_);
        ret = sp_open(serial_port_, SP_MODE_READ_WRITE);
        if (ret != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Start serial_node");
    }

    ~SerialNode() {
        if (serial_port_ != nullptr) {
            sp_close(serial_port_);
        }
    }

private:
    void socketTopicCallback(const socket_communication::msg::Opencv1::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received opencv_topic message: data_flag=%d, flag_condition=%d", msg->data_flag, msg->flag_condition);
        std::string data_to_send = std::to_string(msg->data_flag) + ";" + std::to_string(msg->flag_condition) + "\n";
        sendToSerial(data_to_send);
    }

    void sendToSerial(const std::string& data) {
        sp_nonblocking_write(serial_port_, data.c_str(), data.length());
    }

    rclcpp::Subscription<socket_communication::msg::Opencv1>::SharedPtr subscription_;
    sp_port *serial_port_;
    std::thread read_thread_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto serial_node = std::make_shared<SerialNode>();
    auto opencv_node = std::make_shared<OpencvNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(serial_node);
    executor.add_node(opencv_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}