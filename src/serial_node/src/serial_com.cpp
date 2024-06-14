#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "socket_communication/msg/opencv1.hpp"
#include <libserialport.h>

sp_return ret;

class ArduinoSubscriber : public rclcpp::Node
{
public:
    ArduinoSubscriber() : Node("arduino_subscriber"), serial_port_(nullptr)
    {
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
        if (ret != SP_OK)
        {
            // Log error
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            return;
        }

        // Start reading from serial port
        read_thread_ = std::thread(&ArduinoSubscriber::readSerial, this);

        // Log success
        RCLCPP_INFO(this->get_logger(), "Start arduino_subscriber node");
    }

    ~ArduinoSubscriber()
    {
        if (serial_port_ != nullptr)
        {
            sp_close(serial_port_);
        }
    }

private:
    void readSerial()
    {
        while (rclcpp::ok())
        {
            char buffer[10];
            int bytes_read = sp_blocking_read(serial_port_, buffer, sizeof(buffer), 0);
            if (bytes_read > 0)
            {
                int value;
                sscanf(buffer, "%d", &value);
                std_msgs::msg::Int32 msg;
                msg.data = value;
                publisher_->publish(msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void socketTopicCallback(const socket_communication::msg::Opencv1::SharedPtr msg)
    {
        // Process messages from socket_topic
        RCLCPP_INFO(this->get_logger(), "Received socket_topic message: data_flag=%d, flag_condition=%d", msg->data_flag, msg->flag_condition);
        // Example: Send data_flag to Arduino (replace with your logic)
        sendToArduino(msg->data_flag);
    }

    void sendToArduino(int value)
    {
        // Example: Convert int to string and send over serial
        std::string str_value = std::to_string(value);
        sp_nonblocking_write(serial_port_, str_value.c_str(), str_value.length());
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<socket_communication::msg::Opencv1>::SharedPtr subscription_;
    sp_port *serial_port_;
    std::thread read_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoSubscriber>());
    rclcpp::shutdown();
    return 0;
}
