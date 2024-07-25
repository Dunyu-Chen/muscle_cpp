#ifndef MUSCLE_CPP_LIB_SIGNAL_H
#define MUSCLE_CPP_LIB_SIGNAL_H

#include "Eigen/Dense"
#include "chrono"
#include "vector"
#include "cmath"

class Sample_Time {
public:
    //构造函数
    Sample_Time(double Period);
    //公共属性
    double period;
    std::chrono::microseconds chrono_us{};
    std::chrono::milliseconds chrono_ms{};
};

class Signal {
public:
    Signal(double Sample_period,int Dof = 1);

    Sample_Time sample_time;
    double global_time;
    int sequence;
    Eigen::VectorXd state;
    Eigen::VectorXd state_dot;

    void reset();
    virtual Eigen::VectorXd explicit_search(double Time);
    virtual void step();
};

class System {
public:
    //构造函数
    System(double Sample_period, int State_dim, int Input_dim);
    //公共属性
    Sample_Time sample_time;
    double global_time;
    int sequence;
    int input_dim;
    int state_dim;
    Eigen::VectorXd state_vector;
    Eigen::VectorXd input_vector;
    // 公共调用方法
    void reset(Eigen::VectorXd Init_state,Eigen::VectorXd Init_input);
    virtual void step(Eigen::VectorXd Input);
};

class Linear_System : public System{
public:
    Linear_System(double Sample_time, int State_dim, int Input_dim);
    void step(Eigen::VectorXd Input) override;
    void set_A_matrix();
    void set_B_matrix();
private:
    Eigen::MatrixXd A_matrix;
    Eigen::MatrixXd B_matrix;
};
#endif //MUSCLE_CPP_LIB_SIGNAL_H
