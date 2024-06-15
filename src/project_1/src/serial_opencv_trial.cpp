#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "socket_communication/msg/opencv1.hpp"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <libserialport.h>

using namespace std::chrono_literals;

sp_return ret;

class SocketNode : public rclcpp::Node {
public:
    SocketNode() : Node("socket_node") {
        publisher_ = this->create_publisher<socket_communication::msg::Opencv1>("socket_topic", 10);

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

        std::thread(&SocketNode::publishMessages, this).detach();
    }

private:
    void publishMessages() {
        while (rclcpp::ok()) {
            char buffer[1024] = {0};
            int valread = read(client_socket_, buffer, 1024);
            if (valread <= 0) {
                break;
            }

            // Parse the received data
            int data_1, data_2;
            sscanf(buffer, "%d;%d", &data_1, &data_2);

            // Create a custom message
            auto msg = socket_communication::msg::Opencv1();
            msg.data_flag = data_1;
            msg.flag_condition = data_2;

            // Publish the message
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: data_1=%d, data_2=%d", msg.data_flag, msg.flag_condition);
        }
    }

    int server_socket_;
    int client_socket_;
    struct sockaddr_in server_address_;
    rclcpp::Publisher<socket_communication::msg::Opencv1>::SharedPtr publisher_;
};

class ArduinoSubscriber : public rclcpp::Node {
public:
    ArduinoSubscriber() : Node("arduino_subscriber"), serial_port_(nullptr) {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("arduino_data", 10);

        // Create subscription to custom message topic
        subscription_ = this->create_subscription<socket_communication::msg::Opencv1>(
            "socket_topic", 10, std::bind(&ArduinoSubscriber::socketTopicCallback, this, std::placeholders::_1));

        // Open serial port (assuming this part is relevant for your application)
        sp_get_port_by_name("/dev/serial/by-path/pci-0000:06:00.4-usb-0:2:1.0-port0", &serial_port_);
        sp_set_baudrate(serial_port_, 9600);
        sp_set_bits(serial_port_, 8);
        sp_set_parity(serial_port_, SP_PARITY_NONE);
        sp_set_stopbits(serial_port_, 1);
        sp_close(serial_port_);
        ret = sp_open(serial_port_, SP_MODE_READ_WRITE);
        if (ret != SP_OK) {
            // Log error
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            return;
        }

        // Start reading from serial port
        read_thread_ = std::thread(&ArduinoSubscriber::readSerial, this);

        // Log success
        RCLCPP_INFO(this->get_logger(), "Start arduino_subscriber node");
    }

    ~ArduinoSubscriber() {
        if (serial_port_ != nullptr) {
            sp_close(serial_port_);
        }
    }

private:
    void readSerial() {
        while (rclcpp::ok()) {
            char buffer[10];
            int bytes_read = sp_blocking_read(serial_port_, buffer, sizeof(buffer), 0);
            if (bytes_read > 0) {
                int value;
                sscanf(buffer, "%d", &value);
                std_msgs::msg::Int32 msg;
                msg.data = value;
                publisher_->publish(msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void socketTopicCallback(const socket_communication::msg::Opencv1::SharedPtr msg) {
        // Process messages from socket_topic
        RCLCPP_INFO(this->get_logger(), "Received socket_topic message: data_flag=%d, flag_condition=%d", msg->data_flag, msg->flag_condition);
        // Example: Send data_flag to Arduino (replace with your logic)
        sendToArduino(msg->data_flag);
    }

    void sendToArduino(int value) {
        // Example: Convert int to string and send over serial
        std::string str_value = std::to_string(value);
        sp_nonblocking_write(serial_port_, str_value.c_str(), str_value.length());
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<socket_communication::msg::Opencv1>::SharedPtr subscription_;
    sp_port *serial_port_;
    std::thread read_thread_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto arduino_subscriber_node = std::make_shared<ArduinoSubscriber>();
    auto socket_node = std::make_shared<SocketNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arduino_subscriber_node);
    executor.add_node(socket_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
