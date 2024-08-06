//
// Created by dunyu on 24-7-8.
//
//
// Created by dunyu on 24-7-8.
//

#include <memory>
#include <chrono> // 这里用来配置时间间隔变量

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "muscle_cpp/lib_signal.h"

class PID_controller : public rclcpp::Node{
public:
    PID_controller(rclcpp::QoS Publisher_QOS, rclcpp::QoS Subscriber_QOS, double Sample_period)
        : Node("PID_controller"),
          my_controller(Sample_period)
    {
        subscriber_target = this ->create_subscription<std_msgs::msg::Float64>("target",Subscriber_QOS,std::bind(&PID_controller::sub_target_callback, this, std::placeholders::_1));
        subscriber_state = this ->create_subscription<std_msgs::msg::Float64>("state",Subscriber_QOS,std::bind(&PID_controller::sub_state_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("control",Publisher_QOS);
        timer_ = this->create_wall_timer(my_controller.sample_time.chrono_ms, std::bind(&PID_controller::timer_callback, this));
    }
    PID_Incremental my_controller;
    std_msgs::msg::Float64 pub_msg;
    //std_msgs::msg::Float64 state_msg;
    //std_msgs::msg::Float64 target_msg;
    Eigen::Vector<double,1> state_vector;
    Eigen::Vector<double,1> target_vector;
    rclcpp::Clock clock;
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_target;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_state;
    rclcpp::TimerBase::SharedPtr timer_;

    void sub_target_callback(const std_msgs::msg::Float64 & msg) {
        target_vector[0] = msg.data;
    }

    void sub_state_callback(const std_msgs::msg::Float64 & msg) {
        state_vector[0] = msg.data;
    }

    void timer_callback(){
        my_controller.observe(target_vector,state_vector);
        pub_msg.data = my_controller.step()[0];
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
    double sample_period = 0.001;
    auto node = std::make_shared<PID_controller>(10,10,sample_period);
    node->my_controller.set_params(500,0.01,80000);
    rclcpp::spin(node);
    rclcpp::shutdown();
}