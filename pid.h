#ifndef _PID_H_
#define _PID_H_

#define PID_MAX_CONTROLLERS 10

typedef struct {
  float Kp;
  float Ki; 
  float Kd;
  float target;
  float measured;
  float error;
  float last_error;
  float integral;
  float derivative;
  float output;
  float output_max;
  float output_min;
  int controller_id;
} PIDController;

void PIDInit(PIDController* controller, int id);
void PIDSetParameters(PIDController* controller, float Kp, float Ki, float Kd); 
void PIDSetOutputLimits(PIDController* controller, float min, float max);
void PIDCalculate(PIDController* controller);

#endif
