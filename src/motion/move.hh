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
    float output0 = alpha_n_.dot(v) + w * radius;
    float output1 = beta_n_.dot(v) + w * radius;
    float output2 = gamma_n_.dot(v) + w * radius;

    motor_alpha_.rotate_closed_loop(output0);
    motor_beta_.rotate_closed_loop(output1);
    motor_gamma_.rotate_closed_loop(output2);
  }

private:
  dev::Motor &motor_alpha_;
  dev::Motor &motor_beta_;
  dev::Motor &motor_gamma_;

  const static inline util::Vector2f alpha_n_{0.5, 0.866};
  const static inline util::Vector2f beta_n_{0.5, -0.866};
  const static inline util::Vector2f gamma_n_{-1, 0};

  constexpr static inline float radius = 0.13; // m
};