#include "muscle_cpp/lib_muscle.h"

// library for pneumatic artificial muscle

Binary_Valve::Binary_Valve(double B_value, double C_value) {
    this-> critical_ratio = B_value;
    this-> sonic_conductance = C_value;
}

double Binary_Valve::get_mass_flow_rate(double Upstream_pressure, double Downstream_pressure) {
    double pressure_ratio = Downstream_pressure / Upstream_pressure;
    double mass_flow_rate = 0.0;
    if (pressure_ratio >critical_ratio)
    {
        mass_flow_rate = Upstream_pressure * sonic_conductance * air_density * sqrt(reference_temperature / supply_temperature) * sqrt(1 - pow(((pressure_ratio - critical_ratio) / (1 - critical_ratio)), 2));
    }
    else if (pressure_ratio<=critical_ratio)
    {
        mass_flow_rate = Upstream_pressure * sonic_conductance * air_density * sqrt(reference_temperature / supply_temperature);
    }
    else{ while(true){}}
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

