//
// Created by dunyu on 24-7-8.
//

#include <memory>
#include <chrono> // 这里用来配置时间间隔变量

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "muscle_cpp/lib_signal.h"

class Linear_simulator : public rclcpp::Node{
public:
    Linear_simulator(rclcpp::QoS Publisher_QOS,rclcpp::QoS Subscriber_QOS,double Sample_period)
    : Node("Linear_system_simulator"),
      my_system(Sample_period, 1, 2)
    {
        subscriber_ = this ->create_subscription<std_msgs::msg::Float64>("control",Subscriber_QOS,std::bind(&Linear_simulator::sub_callback,this,std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("state",Publisher_QOS);
        timer_ = this->create_wall_timer(this->my_system.sample_time.chrono_ms, std::bind(&Linear_simulator::timer_callback, this));
    }
    Linear_system my_system;
    std_msgs::msg::Float64 pub_msg;
    std_msgs::msg::Float64 sub_msg;
    rclcpp::Clock clock;
    Eigen::Vector<double,1> input_vector;
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    void sub_callback(const std_msgs::msg::Float64 & msg) {
        sub_msg.data = msg.data;
        input_vector[0] = msg.data;
    }

    void timer_callback(){
        my_system.explicit_step(input_vector);
        pub_msg.data = my_system.output_vector[0];
        publisher_->publish(pub_msg);
    }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::QoS pub_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    pub_qos.history(rclcpp::HistoryPolicy::KeepLast);
    pub_qos.keep_last(5);
    pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    rclcpp::QoS sub_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    sub_qos = pub_qos;
    // create node

    auto node = std::make_shared<Linear_simulator>(10, 10, 0.001);

    double m = 0.01;
    double k = 0.1;
    double b = 0.01;
    double a_list[] = {0, -k / m, 1, -b / m};
    node->my_system.set_A_matrix_from_list(a_list);
    double b_list[] = {0, 1 / m};
    node->my_system.set_B_matrix_from_list(b_list);
    rclcpp::spin(node);
    rclcpp::shutdown();

}