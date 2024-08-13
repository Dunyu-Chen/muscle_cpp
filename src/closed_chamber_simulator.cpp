
#include "muscle_cpp/lib_muscle.h"
#include <Eigen/Dense>
#include <vector>
#include "muscle_cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
int main(){
    // constants
    double Supply_Pressure = 500.0 ;//kpa
    double Atmosphere_Pressure = 100.0; //kpa
    double Reference_Temp = 293.15 ;//K
    double Supply_Temp = 293.15 ; //K
    double Air_Density = 1.185 ;//kg/m^3
    //double C = 0.4; // L/s/bar
    double C = 0.4*0.00001; // m^3/s/kpa
    double B = 0.38 ;
    //double Gas_Constant = 287 ; // J/kg/K = pa*m^2/kg/K
    double Gas_Constant = 287 * 0.001; // kpa*m^2/kg/K
    double Polytropic_Constant = 1.15;
    double Volume = 0.001*1; // m^3
    Close_Chamber chamber(0.001,B,C);
    chamber.set_volume(Volume);
    chamber.set_supply_pressure(Supply_Pressure);
    chamber.set_atmosphere_pressure(Atmosphere_Pressure);
    chamber.set_R(Polytropic_Constant);
    chamber.inlet_valve;

    Eigen::Vector<double,1> Init_State;
    Eigen::Vector<double,2> Init_Input;
    Init_Input(0) =1; Init_Input(1) =0;
    Init_State(0) = 250;
    std::vector<double> state;
    std::vector<double> time;
    chamber.reset(Init_State,Init_Input);
    state.emplace_back(chamber.state_vector(0));
    time.emplace_back(chamber.global_time);
    for (int i =0;i<100;i++){
        if (i%10<3.3)
        {
            Init_Input(0) = 1;
            Init_Input(1) = 0;
        }
        else if (i%10>=3.3&&i%10<6.6)
        {
            Init_Input(0) = 0;
            Init_Input(1) = 0;
        }
        else
        {
            Init_Input(0) = 0;
            Init_Input(1) = 1;
        }
        chamber.step(Init_Input);
        state.emplace_back(chamber.state_vector(0));
        time.emplace_back(chamber.global_time);
    }

    plt::plot(time,state);
    plt::show();


}