//
// Created by dunyu on 24-7-29.
//

#ifndef MUSCLE_CPP_LIB_MUSCLE_H
#define MUSCLE_CPP_LIB_MUSCLE_H
#include <cmath>
class Binary_Valve {
public:
    Binary_Valve(double B_value, double C_value);
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

#endif //MUSCLE_CPP_LIB_MUSCLE_H
