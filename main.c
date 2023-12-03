#include "pid.h"

int main(){

  PIDController controller1, controller2;

  PIDInit(&controller1, 0);
  PIDInit(&controller2, 1);

  PIDSetParameters(&controller1, 1, 0.5, 0.1);

  while(1){
  
    // get measured value
    controller1.measured = read_sensor();  

    // calculate PID output
    PIDCalculate(&controller1);

    // apply output
    apply_output(controller1.output);

  }

  return 0;
}
