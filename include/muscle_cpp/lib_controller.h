//
// Created by dunyu on 24-6-21.
//

#ifndef MUSCLE_CPP_LIB_CONTROLLER_H
#define MUSCLE_CPP_LIB_CONTROLLER_H
#include "vector"
#include "muscle_cpp/lib_signal.h"
#include <Eigen/Dense>

class Discrete_Controller {
public:
    Discrete_Controller();
    std::vector<double> state_vector;
    std::vector<double> target_vector;
    std::vector<double> output_vector;
    void observe(std::vector<double> State, std::vector<double> Target);
    virtual std::vector<double> step();
};

struct PID_Params{
    double P;
    double I;
    double D;
};
class PID_incremental : public Discrete_Controller {
public:
    PID_incremental(PID_Params Params) ;
    std::vector<double> error_buffer;
    std::vector<double> output_buffer;
    PID_Params params;
    std::vector<double> step() override;
};

class Linear_System {
public:
    Eigen::Vector<double,Eigen::Dynamic> state_vector;
    Eigen::Vector<double,Eigen::Dynamic> input_vector;
    Linear_System(int State_dim, int Input_dim);
    void set_A_matrix_from_list(double* Element_list);
    void set_B_matrix_from_list(double* Element_list);
private:
    int state_dim;
    int input_dim;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A_matrix;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> B_matrix;
};

struct MSD_Params{
    double M;
    double S;
    double D;
};


#endif //MUSCLE_CPP_LIB_CONTROLLER_H
