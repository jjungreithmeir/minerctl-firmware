#ifndef AUTOPID_H
#define AUTOPID_H
#include <Arduino.h>

class AutoPID {

  public:
    // Constructor - takes pointer inputs for control variales, so they are updated automatically
    AutoPID(int *input, int *setpoint, int *output, int outputMin, int outputMax,
            int Kp, int Ki, int Kd);
    // Allows manual adjustment of gains
    void setGains(int Kp, int Ki, int Kd);
    // Sets bang-bang control ranges, separate upper and lower offsets, zero for off
    void setBangBang(int bangOn, int bangOff);
    // Sets bang-bang control range +-single offset
    void setBangBang(int bangRange);
    // Allows manual readjustment of output range
    void setOutputRange(int outputMin, int outputMax);
    // Allows manual adjustment of time step (default 1000ms)
    void setTimeStep(unsigned long timeStep);
    // Returns true when at set point (+-threshold)
    bool atSetPoint(int threshold);
    // Runs PID calculations when needed. Should be called repeatedly in loop.
    // Automatically reads input and sets output via pointers
    void run();
    // Stops PID functionality, output sets to 
    void stop();
    void reset();
    bool isStopped();

    int getIntegral();
    void setIntegral(int integral);

  private:
    int _Kp, _Ki, _Kd;
    int _integral, _previousError;
    int _bangOn, _bangOff;
    int *_input, *_setpoint, *_output;
    int _outputMin, _outputMax;
    unsigned long _timeStep, _lastStep;
    int _stopped;

};//class AutoPID

class AutoPIDRelay : public AutoPID {
  public:

    AutoPIDRelay(int *input, int *setpoint, bool *relayState, int pulseWidth, int Kp, int Ki, int Kd)
      : AutoPID(input, setpoint, &_pulseValue, 0, 1.0, Kp, Ki, Kd) {
      _relayState = relayState;
      _pulseWidth = pulseWidth;
    };

    void run();

    int getPulseValue();

  private:
    bool * _relayState;
    unsigned long _pulseWidth, _lastPulseTime;
    int _pulseValue;
};//class AutoPIDRelay

#endif
