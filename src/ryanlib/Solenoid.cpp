#include "Solenoid.hpp"

namespace ryan{

Solenoid::Solenoid(std::uint8_t iPort, bool initState) : piston(iPort), state(initState){
    piston.set_value(state);
}

Solenoid::Solenoid(pros::ext_adi_port_pair_t iPortPair, bool initState) : piston(iPortPair), state(initState) {
    piston.set_value(state);
}

void Solenoid::toggle(){
    state = !state;
    piston.set_value(state);
}

void Solenoid::set(bool iState){
    state = iState;
    piston.set_value(iState);
}

bool Solenoid::getState() const{
    return state;
}

}