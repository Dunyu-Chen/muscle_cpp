#include "muscle_cpp/lib_muscle.h"

// library for pneumatic artificial muscle

Binary_Valve::Binary_Valve(double B_value, double C_value,double Temp_ref,double Temp_supply,double Air_density) {
    this-> critical_ratio = B_value;
    this-> sonic_conductance = C_value;
    reference_temperature = Temp_ref;
    supply_temperature = Temp_supply;
    air_density = Air_density;
}

double Binary_Valve::get_mass_flow_rate(double Upstream_pressure, double Downstream_pressure) {
    double pressure_ratio = Downstream_pressure / Upstream_pressure;
    double mass_flow_rate = 0.0;
    if (pressure_ratio >=1){
        pressure_ratio = 1;
    }
    else if (pressure_ratio >critical_ratio)
    {
        mass_flow_rate = Upstream_pressure * sonic_conductance * air_density * sqrt(reference_temperature / supply_temperature) * sqrt(1 - pow(((pressure_ratio - critical_ratio) / (1 - critical_ratio)), 2));
    }
    else if (pressure_ratio<=critical_ratio)
    {
        mass_flow_rate = Upstream_pressure * sonic_conductance * air_density * sqrt(reference_temperature / supply_temperature);
    }

    return mass_flow_rate;
}

void Binary_Valve::set_air_density(double Density) {
    air_density = Density;
}

void Binary_Valve::set_ref_temperature(double Temperature) {
    reference_temperature = Temperature;
}

void Binary_Valve::set_supply_temperature(double Temperature) {
    supply_temperature = Temperature;
}

Close_Chamber::Close_Chamber(double Sample_period,double Valve_B_value,double Valve_C_value,double Gas_constant,double Temp_supply)
    : System(Sample_period,1,2),
      inlet_valve(Valve_B_value,Valve_C_value),
      outlet_valve(Valve_B_value,Valve_C_value)
{
    process_parameter = 1;
    gas_constant = Gas_constant;
    supply_temperature = Temp_supply;
    dot_state.resize(state_dim,1);
    B_matrix.resize(input_dim,state_dim);
}

void Close_Chamber::set_supply_temperature(double Temp) {
    supply_temperature = Temp;
}

void Close_Chamber::set_R(double R_value) {
    process_parameter =  R_value;
}

void Close_Chamber::set_alpha(double Alpha) {
    if (Alpha>1.4)
        {process_parameter = 1.4;}
    else if (Alpha<1)
        {process_parameter = 1;}
    else
        {process_parameter = Alpha;}
}
void Close_Chamber::set_atmosphere_pressure(double Pressure) {
    this->atmosphere_pressure = Pressure;
}
void Close_Chamber::set_supply_pressure(double Pressure) {
    this->supply_pressure = Pressure;
}
void Close_Chamber::set_volume(double Volume) {
    this->volume = Volume;
}
Eigen::MatrixXd Close_Chamber::update_B_matrix(Eigen::MatrixXd State) {
    double param = process_parameter*(this->gas_constant*supply_temperature)/volume;
    double pressure = State(0);
    Eigen::MatrixXd return_matrix;
    return_matrix.resize(state_dim,input_dim);
    return_matrix(0,0) = param*inlet_valve.get_mass_flow_rate(supply_pressure,pressure);
    return_matrix(0,1) = -param*outlet_valve.get_mass_flow_rate(pressure,atmosphere_pressure);
    return return_matrix;
}
void Close_Chamber::step(Eigen::VectorXd Input) {
    System::step(Input);
    B_matrix = update_B_matrix(state_vector).matrix();
    dot_state = B_matrix*input_vector;
    //explicit step
    state_vector = (state_vector + sample_time.period*dot_state).matrix();
}

