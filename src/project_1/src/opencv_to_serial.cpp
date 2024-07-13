#include "rclcpp/rclcpp.hpp"
#include "socket_communication/msg/opencv1.hpp"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <libserialport.h>
#include <string>
#include <thread>

using namespace std::chrono_literals;

int server_socket_;
int client_socket_;
struct sockaddr_in server_address_;
sp_port *serial_port_ = nullptr;
rclcpp::Publisher<socket_communication::msg::Opencv1>::SharedPtr publisher_;
rclcpp::Subscription<socket_communication::msg::Opencv1>::SharedPtr subscription_;
rclcpp::Node::SharedPtr opencv_node;
rclcpp::Node::SharedPtr serial_node;

int data_flag;
int flag_condition;
int warning;

void setupSocket() {
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
}

void publishMessages() {
    char buffer[1024] = {0};
    while (rclcpp::ok()) {
        int valread = read(client_socket_, buffer, 1024);
        if (valread <= 0) break;

        int data_1, data_2;
        sscanf(buffer, "%d;%d", &data_1, &data_2);

        auto msg = socket_communication::msg::Opencv1();
        msg.data_flag = data_1;
        msg.flag_condition = data_2;

        publisher_->publish(msg);
        //RCLCPP_INFO(rclcpp::get_logger("opencv_node"), "Publishing: data_1=%d, data_2=%d", msg.data_flag, msg.flag_condition);
    }
}

void setupSerialPort() {
    sp_get_port_by_name("/dev/serial/by-path/pci-0000:06:00.4-usb-0:2:1.0-port0", &serial_port_);
    sp_set_baudrate(serial_port_, 9600);
    sp_set_bits(serial_port_, 8);
    sp_set_parity(serial_port_, SP_PARITY_NONE);
    sp_set_stopbits(serial_port_, 1);
    if (sp_open(serial_port_, SP_MODE_READ_WRITE) != SP_OK) {
        RCLCPP_ERROR(rclcpp::get_logger("serial_node"), "Failed to open serial port.");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("serial_node"), "Serial port opened");
}

void socketTopicCallback(const socket_communication::msg::Opencv1::SharedPtr msg) {
    //RCLCPP_INFO(rclcpp::get_logger("serial_node"), "Received opencv_topic message: data_flag=%d, flag_condition=%d", msg->data_flag, msg->flag_condition);
    data_flag = msg->data_flag;
    flag_condition = msg->flag_condition;
    //std::string data_to_send = std::to_string(msg->data_flag) + ";" + std::to_string(msg->flag_condition) + "\n";
    //sp_nonblocking_write(serial_port_, data_to_send.c_str(), data_to_send.length());
    //std::this_thread::sleep_for(std::chrono::seconds(1)); testing for running multi thread
}

void startOpencvNode() {
    opencv_node = rclcpp::Node::make_shared("opencv_node");
    publisher_ = opencv_node->create_publisher<socket_communication::msg::Opencv1>("opencv_topic", 10);
    setupSocket();
    std::thread(publishMessages).detach();
}

void startSerialNode() {
    serial_node = rclcpp::Node::make_shared("serial_node");
    subscription_ = serial_node->create_subscription<socket_communication::msg::Opencv1>(
        "opencv_topic", 10, socketTopicCallback);
    setupSerialPort();
}

void thread1() {
    if (data_flag == 2 && flag_condition == 2) {
    warning = 3;
    } else if (data_flag == 2 && flag_condition == 0) {
    warning = 2;
    } else if (data_flag == 0 && flag_condition == 2) {
    warning = 1;
    } else {
    warning = 0;
    }
    //printf("Thread 1\n");
}

void thread2() {
    std::string data_to_send = std::to_string(warning) + "\n"; // "1\n"
    sp_nonblocking_write(serial_port_, data_to_send.c_str(), data_to_send.length());
    //printf("      Thread 2 || %d\n", warning);
    printf(data_to_send.c_str());
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    startOpencvNode();
    startSerialNode();

    auto timer1 = serial_node->create_wall_timer(std::chrono::milliseconds(100), thread1);
    auto timer2 = serial_node->create_wall_timer(std::chrono::milliseconds(500), thread2);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(serial_node);
    executor.add_node(opencv_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
