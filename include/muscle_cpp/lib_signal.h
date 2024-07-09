#ifndef MUSCLE_CPP_LIB_SIGNAL_H
#define MUSCLE_CPP_LIB_SIGNAL_H

#include "Eigen/Dense"
#include "chrono"
#include "vector"
#include "cmath"

struct Node_Sample_Time{
    double period; // in s
    std::chrono::microseconds chrono_us{};
    std::chrono::milliseconds chrono_ms{};
};
Node_Sample_Time init_node_sample_time(double Period);
class Signal {
public:
    Signal(double Sample_time,int Input_dim,int Output_dim);

    Node_Sample_Time sample_time;
    double Time_global;
    int sequence;
    Eigen::VectorXd output_vector;
    Eigen::VectorXd input_vector;

    void reset(Eigen::VectorXd init_state);
    virtual void step(Eigen::VectorXd Input);
};

struct Sin_Params {
    double Amp;
    double Fre;
    double Mar;
    double Pha;
};
class Signal_sin : public Signal{
public:
    Signal_sin(double Sample_time,Sin_Params Params);

    Sin_Params params{};
    void step(Eigen::VectorXd Input) override;
    double search(double Time) const;
};

struct Stair_Params {
    double period;
    std::vector<double> time_list;
    std::vector<double> data_list;
};

class Signal_stair : public Signal{
public:
    Signal_stair(double Sample_time,Stair_Params params) ;

    Stair_Params params;
    double search(double Time);
    void step(Eigen::VectorXd Input) override;
};

struct PID_Params{
    double K_p;
    double K_i;
    double K_d;
};

class PID_Incremental{
public:
    PID_Incremental(PID_Params Params);

    PID_Params params;
    double output_buffer[2];
    double error_buffer[3];
    double coe1;
    double coe2;
    double coe3;

    void reset();
    double step(double Target, double state);
};

class Linear_system : public Signal {
public:
    Linear_system(double Sample_time,int Input_dim,int State_dim);

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A_matrix;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> B_matrix;
    int input_dim;
    int state_dim;

    void set_A_matrix_from_list(double * Data_list);
    void set_B_matrix_from_list(double * Data_list);
    void explicit_step(Eigen::VectorXd Input);
};


#endif //MUSCLE_CPP_LIB_SIGNAL_H
