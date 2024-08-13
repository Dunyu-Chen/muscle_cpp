//
// Created by dunyu on 24-7-29.
//

#ifndef MUSCLE_CPP_LIB_MUSCLE_H
#define MUSCLE_CPP_LIB_MUSCLE_H
#include <cmath>
#include "muscle_cpp/lib_signal.h"

class Binary_Valve {
public:
    Binary_Valve(double B_value, double C_value,double Temp_ref=293.15 ,double Temp_supply = 293.15,double Air_density =1.185);
    double critical_ratio;
    double sonic_conductance;
    double reference_temperature;
    double supply_temperature;
    double air_density;
    void set_air_density(double Density);
    void set_ref_temperature(double Temperature);
    void set_supply_temperature(double Temperature);
    double get_mass_flow_rate(double Upstream_pressure, double Downstream_pressure);
};

class Close_Chamber : public System{
public:
    Close_Chamber(double Sample_period,double Valve_B_value,double Valve_C_value,double Gas_constant= 287 * 0.001,double Temp_supply = 293.15);

    Binary_Valve inlet_valve;
    Binary_Valve outlet_valve;
    double gas_constant;
    double supply_temperature;
    double process_parameter;
    double supply_pressure;
    double atmosphere_pressure;
    double volume;
    Eigen::MatrixXd B_matrix;
    Eigen::MatrixXd dot_state;

    Eigen::MatrixXd update_B_matrix(Eigen::MatrixXd State);
    void set_volume(double Volume);
    void set_R(double R);
    void set_alpha(double alpha);
    void set_supply_temperature(double Temp);
    void set_supply_pressure(double Pressure);
    void set_atmosphere_pressure(double Pressure);

    void step(Eigen::VectorXd Input) override;
};

class Close_Chamber_Opt_Controller : public Controller {
public:

};

#endif //MUSCLE_CPP_LIB_MUSCLE_H
