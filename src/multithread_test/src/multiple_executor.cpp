#include <chrono>
#include "rclcpp/rclcpp.hpp"

int a = 0;

void thread1()
{
  printf("Thread 1\n");
  //delay 1 s
  std::this_thread::sleep_for(std::chrono::seconds(2));
  a++;
}

void thread2()
{  
  printf("      Thread 2 || %d\n", a);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("thread1_tutor");
    auto node2 = rclcpp::Node::make_shared("thread2_tutor");
  
    auto timer1 = node->create_wall_timer(std::chrono::milliseconds(100), thread1);
    auto timer2 = node2->create_wall_timer(std::chrono::milliseconds(1000), thread2);

    //single exceutor
    // rclcpp::executors::SingleThreadedExecutor executor;


    //multi executor
    rclcpp::executors::MultiThreadedExecutor executor;


    executor.add_node(node);
    executor.add_node(node2);

    executor.spin();
    rclcpp::shutdown();

    return 0;

}