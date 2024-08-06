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

Signal_Sin::Signal_Sin(double Sample_period, Sin_Params Params)
    : Signal(Sample_period)
{
    params = Params;
}

Eigen::VectorXd Signal_Sin::explicit_search(double Time) {
    Eigen::VectorXd return_value(this->state.size());
    return_value[0] = params.Amp*sin(params.Fre*Time+params.Pha)+params.Mar;
    return return_value;
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

void System::reset(Eigen::VectorXd Init_state, Eigen::VectorXd Init_input) {
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

void Linear_System::set_A_matrix(Eigen::MatrixXd Target) {
    if ((Target.cols() == A_matrix.cols())&&(Target.rows() == A_matrix.rows())){
        A_matrix = Target.matrix();
    }
}

void Linear_System::set_B_matrix(Eigen::MatrixXd Target) {
    if ((Target.cols() == B_matrix.cols())&&(Target.rows() == B_matrix.rows())){
        B_matrix = Target.matrix();
    }
}

Controller::Controller(double Sample_time, int State_dim, int Output_dim)
    : sample_time(Sample_time)
{
    global_time = 0.0;
    sequence = 0.0;
    state_dim = State_dim;
    output_dim = Output_dim;
    state_vector.resize(State_dim);
    target_vector.resize(State_dim);
    output_vector.resize(Output_dim);
    state_vector.setZero();
    target_vector.setZero();
    output_vector.setZero();
}

void Controller::observe(Eigen::VectorXd Target, Eigen::VectorXd State) {
    if ((Target.size() ==this->state_dim)&&(State.size()== this->state_dim) ){
        target_vector = Target.matrix();
        state_vector = State.matrix();
    }
    else{
        while (1){};
    }
}

Eigen::VectorXd Controller::step() {
    global_time +=sample_time.period;
    sequence +=1;
    return output_vector;
}

PID_Incremental::PID_Incremental(double Sample_time)
    : Controller(Sample_time,1,1) // discrete controller
{
    error_buffer.setZero();
    feed_back_matrix.setZero();
}

void PID_Incremental::set_params(double P,double I,double D){
    k_p = P;
    k_i = I;
    k_d = D;
    feed_back_matrix[0] = k_p + k_i + k_d;
    feed_back_matrix[1] = -k_p - 2 * k_d;
    feed_back_matrix[2] = k_d;
}

Eigen::VectorXd PID_Incremental::step() {
    error_buffer.row(2) = error_buffer.row(1);
    error_buffer.row(1) = error_buffer.row(0);
    error_buffer.row(0) = target_vector - state_vector;
    output_vector = output_vector.matrix() + feed_back_matrix * error_buffer;
    return Controller::step();
}