// socket_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

using namespace std::chrono_literals;

class SocketNode : public rclcpp::Node {
public:
    SocketNode() : Node("socket_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("socket_topic", 10);

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
            auto msg = std_msgs::msg::String();
            msg.data = buffer;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        }
    }

    int server_socket_;
    int client_socket_;
    struct sockaddr_in server_address_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
