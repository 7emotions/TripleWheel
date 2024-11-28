#pragma once

#include "dev/motor/motor.hh"
#include "util/math.hh"

class Motion {
public:
  /**
   * @brief Construct a new Motion object
   *
   * @param motor_alpha theta = pi / 2
   * @param motor_beta theta = 7 pi / 6
   * @param motor_gamma theta = 11 pi / 6
   */
  Motion(dev::Motor &motor_alpha, dev::Motor &motor_beta,
         dev::Motor &motor_gamma)
      : motor_alpha_(motor_alpha), motor_beta_(motor_beta),
        motor_gamma_(motor_gamma) {}

  // r: rad/s
  void move(const util::Vector2f &v, float w) {
    float output0 = v.x - v.y - w * (rx + ry);
    float output1 = v.x + v.y + w * (rx + ry);
    float output2 = v.x + v.y - w * (rx + ry);

    motor_alpha_.rotate_open_loop(output0);
    motor_beta_.rotate_open_loop(output1);
    motor_gamma_.rotate_open_loop(output2);
  }

private:
  dev::Motor &motor_alpha_;
  dev::Motor &motor_beta_;
  dev::Motor &motor_gamma_;

  constexpr static inline float rx = 0.525f; // m
  constexpr static inline float ry = 0.950f; // m
};