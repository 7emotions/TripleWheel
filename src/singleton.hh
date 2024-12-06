#pragma once
#include <cstdint>

#include "tim.h"
#include "usart.h"

#include "base/encoder.hh"
#include "base/gpio.hh"
#include "base/pwm.hh"
#include "base/serial.hh"
#include "base/timer.hh"

#include "dev/motor/motor.hh"
#include "motion/move.hh"

constexpr uint16_t encoder_count_limit = 1400;

namespace single {
using namespace hal;

using pin_a0 = gpio::PE<14>;
using pin_a1 = gpio::PE<15>;
using pwm_a = PWM<&htim1, pwm::channel1>;
inline dev::Motor motor0{};

using pin_b0 = gpio::PB<10>;
using pin_b1 = gpio::PB<11>;
using pwm_b = PWM<&htim1, pwm::channel2>;
inline dev::Motor motor1{};

using pin_c0 = gpio::PB<12>;
using pin_c1 = gpio::PB<13>;
using pwm_c = PWM<&htim1, pwm::channel3>;
inline dev::Motor motor2{};

inline Encoder<&htim2> encoder0{};
inline Encoder<&htim3> encoder1{};
inline Encoder<&htim4> encoder2{};

using led = gpio::PA<1>;

using servo_left = PWM<&htim8, pwm::channel1>;
using servo_right = PWM<&htim8, pwm::channel2>;

inline Timer<&htim5> timer5{};

inline Serial<&huart1> remote{};

inline Motion motion{motor0, motor2, motor1};

} // namespace single