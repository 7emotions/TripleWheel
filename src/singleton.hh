#pragma once

#include "base/encoder.hh"
#include "base/gpio.hh"
#include "base/pwm.hh"
#include "base/timer.hh"

#include "dev/motor/motor.hh"

#include "motion/move.hh"

#include <cstdint>

#include "tim.h"
constexpr uint16_t encoder_count_limit = 3100;

namespace single {
using namespace hal;

using pin_a0 = gpio::PA<3>;
using pin_a1 = gpio::PA<2>;
using pwm_a = PWM<&htim1, pwm::channel1>;
inline dev::Motor motor0{}; // right back

using pin_b0 = gpio::PB<1>;
using pin_b1 = gpio::PB<0>;
using pwm_b = PWM<&htim1, pwm::channel2>;
inline dev::Motor motor1{}; // left back

using pin_c0 = gpio::PD<15>;
using pin_c1 = gpio::PD<14>;
using pwm_c = PWM<&htim1, pwm::channel3>;
inline dev::Motor motor2{}; // right front

using pin_d0 = gpio::PC<6>;
using pin_d1 = gpio::PC<7>;
using pwm_d = PWM<&htim1, pwm::channel4>;
inline dev::Motor motor3{}; // left front

// speed0 or speed1 = 500/13 * speed2 or speed3
inline Encoder<&htim2> encoder0{};
inline Encoder<&htim3> encoder1{};

inline Encoder<&htim4> encoder2{};

using led = gpio::PE<3>;

inline Timer<&htim5> timer5{};

inline Motion motion{motor3, motor2, motor1};

} // namespace single