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

struct Sin_Params{
    double Amp;
    double Fre;
    double Pha;
    double Mar;
};

class Signal_Sin : public Signal {
public:
    Signal_Sin(double Sample_period, Sin_Params Params);

    Sin_Params params;

    Eigen::VectorXd explicit_search(double Time) override;
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
    void set_A_matrix(Eigen::MatrixXd Target);
    void set_B_matrix(Eigen::MatrixXd Target);
    Eigen::MatrixXd A_matrix;
    Eigen::MatrixXd B_matrix;
};

class Controller{
public:
    Controller(double Sample_time,int State_dim, int Output_dim);
    Sample_Time sample_time;
    double global_time;
    int sequence;
    int state_dim;
    int output_dim;

    Eigen::VectorXd state_vector;
    Eigen::VectorXd target_vector;
    Eigen::VectorXd output_vector;

    void observe(Eigen::VectorXd Target, Eigen::VectorXd State);
    virtual Eigen::VectorXd step(); // based on observed state of k, get control output of k
};

class PID_Incremental : public Controller{
public:
    PID_Incremental(double Sample_time);

    Eigen::Vector3d error_buffer;
    double k_p;
    double k_i;
    double k_d;
    Eigen::Matrix<double,1,3> feed_back_matrix;

    void set_params(double P,double I,double D);
    Eigen::VectorXd step() override;
};
#endif //MUSCLE_CPP_LIB_SIGNAL_H
