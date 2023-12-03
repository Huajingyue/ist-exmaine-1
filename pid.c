#include "pid.h"

static PIDController controllers[PID_MAX_CONTROLLERS];

void PIDInit(PIDController* controller, int id){
  controller->controller_id = id;
  controller->Kp = 0;
  controller->Ki = 0;
  controller->Kd = 0;
  controller->target = 0;
  controller->measured = 0;
  controller->error = 0; 
  controller->last_error = 0;
  controller->integral = 0;
  controller->derivative = 0;
  controller->output = 0;
  controller->output_max = 1000;
  controller->output_min = -1000;
}

void PIDSetParameters(PIDController* controller, float Kp, float Ki, float Kd){
  controller->Kp = Kp;
  controller->Ki = Ki;
  controller->Kd = Kd;
}

void PIDSetOutputLimits(PIDController* controller, float min, float max){
  controller->output_max = max;
  controller->output_min = min;  
}

void PIDCalculate(PIDController* controller){
  // PID algorithm
  controller->error = controller->target - controller->measured;
  
  controller->integral += controller->error;
  
  controller->derivative = controller->error - controller->last_error;
  
  controller->output = controller->Kp*controller->error  
                      + controller->Ki*controller->integral
                      + controller->Kd*controller->derivative;
                      
  if(controller->output > controller->output_max)
    controller->output = controller->output_max;
  else if(controller->output < controller->output_min)  
    controller->output = controller->output_min;

  controller->last_error = controller->error;
}
