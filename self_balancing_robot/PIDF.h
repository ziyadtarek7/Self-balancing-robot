/*  =============================================
PIDF library for Arduino and STM32 boards.
Author:  N/A.
First release: November 10th, 2021

Changelog:
- 10/11/2021  Launched.

This library contains an implementation for the PIDF controller (Proportional
Integral Derivative with Filter) used by MATLAB and Simulink. To make the best
use of this library, model your system on Simulink and design and tune a
discrete-time PIDF controller, then initialize the PIDF object with the tuning
parameters you obtained.
=============================================  */

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#ifndef PIDF_h
  #define PIDF_h

namespace PID
{
  class PIDF
  {
   public:
    // Constructors
    PIDF(double P, double I, double D, double N, double Ts);

    // Main methods
    double setpoint(void);
    void setpoint(double setpoint);
    double calculate(double processVariable);
    void limitOutput(double outputMin, double outputMax);

    // Modifiers
    void tune(double P, double I, double D, double N);
    double samplingTime(void);
    void samplingTime(double Ts);

    // Debugging helpers
    double _proportional(double error);
    double _integral(double error);
    double _derivative(double error);
    double _filter(double error);

   private:
    double Kp {1.0}, Ki {1.0}, Kd {1.0}, Kn {1.0};
    double Ts {0.2};

    double sp {0.0}, output {0.0};
    double outMin {-65535.0}, outMax {65535.0};

    double integralAccumulator {0}, filterAccumulator {0};

    double initialState {0};
  };
}  // namespace PID
#endif
