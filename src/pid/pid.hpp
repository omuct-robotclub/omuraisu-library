#ifndef PID_NEW_HPP
#define PID_NEW_HPP

#include <algorithm>

struct PidGain {
  float kp;
  float ki;
  float kd;
};

struct PidParameter {
  PidGain gain;
  float min;
  float max;
};

class Pid {
 public:
  Pid(const PidParameter parameter)
      : _parameter(parameter), _pre_error(0), _integral(0) {}

  Pid() : _parameter{PidGain{0, 0, 0}, 0, 0}, _pre_error(0), _integral(0) {}

  float calc(const float goal, const float actual, const float dt_sec) {
    float error = goal - actual;
    _integral += error * dt_sec;
    float deriv = dt_sec == 0 ? 0 : (error - _pre_error) / dt_sec;
    float output = _parameter.gain.kp * error + _parameter.gain.ki * _integral +
                   _parameter.gain.kd * deriv;

    output = std::clamp(output, _parameter.min, _parameter.max);
    _pre_error = error;
    return output;
  }

  void reset() {
    _integral = 0;
    _pre_error = 0;
  }

  void set_gain(const PidGain gain) { _parameter.gain = gain; }

  void set_limit(const float max, const float min) {
    _parameter.max = max;
    _parameter.min = min;
  }

 private:
  PidParameter _parameter;
  float _pre_error;
  float _integral;
};

#endif  // PID_NEW_HPP