#include "muscle_cpp/lib_signal.h"

Sample_Time::Sample_Time(double Period) {
    this->period = Period;
    this->chrono_ms = std::chrono::milliseconds (static_cast<int>(std::round(Period * 1000.0)));
    this->chrono_us = std::chrono::microseconds(static_cast<int>(std::round(Period * 1000000.0)));
}

Signal::Signal(double Sample_period, int Dof)
    : sample_time(Sample_period)
{
    this->global_time = 0.0;
    this->sequence = 0;
    this->state.resize(Dof);
    this->state.setZero();
    this->state_dot.resize(Dof);
    this->state_dot.setZero();
}

void Signal::reset(){
    this->global_time = 0.0;
    this->sequence = 0;
    this->state.setZero();
    this->state_dot.setZero();
}

Eigen::VectorXd Signal::explicit_search(double Time) {
    Eigen::VectorXd return_value(this->state.size());
    return_value.setZero();
    return return_value;
}

void Signal::step() {
    this->global_time += this->sample_time.period;
    this->sequence +=1;
    auto state_buffer = this->state;
    this->state = this->explicit_search(this->global_time);
    state_dot = this->sample_time.period*(this->state - state_buffer);
}

System::System(double Sample_period, int State_dim, int Input_dim)
    : sample_time(Sample_period)
{
    this->global_time = 0.0;
    this->sequence = 0;
    input_dim=Input_dim;
    state_dim=State_dim;
    state_vector.resize(State_dim);
    state_vector.setZero();
    input_vector.resize(Input_dim);
    input_vector.setZero();
}

void System::reset(Eigen::VectorXd Init_state, Eigen::VectorXd Init_input = Eigen::VectorXd::Zero()) {
    this->global_time = 0.0;
    this->sequence = 0;
    if ((Init_state.size() == this->state_dim)&&(Init_input.size() == this->input_dim)){
        state_vector = Init_state.matrix();
        input_vector= Init_input.matrix();
    }
    else{
        while(1){};
    }
}

void System::step(Eigen::VectorXd Input) {
    this->global_time += this->sample_time.period;
    this->sequence +=1;
    if (Input.size() == this->input_dim){
        this->input_vector = Input.matrix();
    }
    else{
        while(1){};
    }
}

Linear_System::Linear_System(double Sample_time, int State_dim, int Input_dim)
    : System(Sample_time,State_dim,Input_dim)
{
    A_matrix.resize(State_dim,State_dim);
    B_matrix.resize(State_dim,Input_dim);
}

void Linear_System::step(Eigen::VectorXd Input) {
    System::step(Input);
    // Euler forward
    state_vector = state_vector.matrix() + (A_matrix*state_vector+B_matrix*input_vector)*sample_time.period;
}

void Linear_System::set_A_matrix() {

}

void Linear_System::set_B_matrix() {

}