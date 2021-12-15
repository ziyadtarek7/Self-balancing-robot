#include "PIDF.h"

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

/**
 * @brief Construct a new PIDF object
 *
 * @param P Proportional gain
 * @param I Integral gain
 * @param D Derivative gain
 * @param N Filter gain
 * @param Ts Sampling time
 */
PID::PIDF::PIDF(double P, double I, double D, double N, double Ts) : Kp(P), Ki(I), Kd(D), Kn(N), Ts(Ts) {};

/**
 * @brief Setpoint getter
 *
 * @return double Current setpoint
 */
double PID::PIDF::setpoint(void)
{
  return sp;
}

/**
 * @brief Setpoint setter; Sets a new setpoint to be maintained
 *
 * @param setpoint New setpoint
 */
void PID::PIDF::setpoint(double setpoint)
{
  if (setpoint <= outMin)
    sp = outMin;
  else if (setpoint >= outMax)
    sp = outMax;
  else
    sp = setpoint;
}

/**
 * @brief Calculates the output for given process variable to maintain the
 * setpoint.
 *
 * @details This is the main function of the PIDF Controller. It uses the three
 * constants Kp, Ki, Kd and Kn to calculate the output for the current value of
 * the process variable.
 *
 * @param processVariable Current value of the process variable
 * @return double PIDF output
 */
double PID::PIDF::calculate(double processVariable)
{
  // calculate the error and de
  double error = sp - processVariable;

  // Calculate each PIDF term separately
  double proportionalTerm = _proportional(error);
  double integralTerm     = _integral(error);
  double derivativeTerm   = _derivative(error);
  double filterTerm       = _filter(error);

  // Calculate the output by summing the 3 Terms
  output = initialState + proportionalTerm + integralTerm + derivativeTerm + filterTerm;

  // Constrain Output_Value
  if (output > outMax)
  {
    output = outMax;
    integralAccumulator -= integralTerm;  // undo accumlation of the Integral Term to prevent the
                                          // accumulator from exploding
  }
  else if (output < outMin)
  {
    output = outMin;
    integralAccumulator -= integralTerm;  // undo accumlation of the Integral Term to prevent the
                                          // accumulator from exploding
  }

  // Update accumulators
  integralAccumulator = integralTerm;
  filterAccumulator   = derivativeTerm;

  return output;
}

/**
 * @brief Sets the output limits used for clamping
 *
 * @param outpunMin Minimum value for the output
 * @param outpunMax Maximum value for the output
 */
void PID::PIDF::limitOutput(double outputMin, double outputMax)
{
  outMin = outputMin;
  outMax = outputMax;
}

/**
 * @brief Modifies the PIDF tuning parameters to alter the controller behaviour
 *
 * @param P Kp value
 * @param I Ki value
 * @param D Kd value
 * @param N Kn value
 */
void PID::PIDF::tune(double P, double I, double D, double N)
{
  integralAccumulator = 0;       // Reset integral term because we will be integrating with a new gain
  initialState        = output;  // Keep previous output value as an initial state for
                                 // the controller

  Kp = P;
  Ki = I;
  Kd = D;
  Kn = N;
}

/**
 * @brief Sampling time getter
 *
 * @return double Ts Sampling time
 */
double PID::PIDF::samplingTime(void)
{
  return Ts;
}

/**
 * @brief Sampling time setter
 *
 * @param double Ts Sampling time
 */
void PID::PIDF::samplingTime(double ts)
{
  PID::PIDF::Ts = ts;
}

/**
 * @brief Calculates the proportional term output
 *
 * @param error Tracking error; setpoint - process variable
 */
double PID::PIDF::_proportional(double error)
{
  return Kp * error;
}

/**
 * @brief Calculates the integral term output
 *
 * @param error Tracking error; setpoint - process variable
 */
double PID::PIDF::_integral(double error)
{
  return integralAccumulator + Ki * Ts * error;
}

/**
 * @brief Calculates the derivative term output
 *
 * @param error Tracking error; setpoint - process variable
 */
double PID::PIDF::_derivative(double error)
{
  return Kd * error + filterAccumulator / (1.0 + Kn * Ts);
}

/**
 * @brief Calculates the filter term output
 *
 * @param error Tracking error; setpoint - process variable
 */
double PID::PIDF::_filter(double error)
{
  return (-filterAccumulator * Kn) / (1.0 + Kn * Ts);
}
