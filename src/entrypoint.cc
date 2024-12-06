#include "entrypoint.hh"
#include "protocol.hh"
#include "singleton.hh"
#include <cstdlib>

using namespace hal;

constexpr auto wheel_num = 3;

namespace global {
float rotation{0.f};
float speed[wheel_num]{0.f};
float v{0.f};

enum class EndState { Up, Down } end_state{EndState::Up};

enum class State { Tracking, Finding, Get_Nearby, Found } state{State::Finding};

auto time{0};

uint8_t recv[15];

protocol::Package package;

uint32_t watchdog_tick{0};
} // namespace global

namespace debug {
int32_t interval[wheel_num];
}

GENERATE_UART_RX_EVENT_CALLBACK(single::remote)
GENERATE_UART_RX_COMPLETE_CALLBACK(single::remote)
GENERATE_TIM_PERIOD_ELAPSED_CALLBACK(single::timer5)

namespace mission {

auto toggle_led() -> void { single::led::toggle(); }

template <global::EndState state> auto set_end_state() -> void {
  if (state == global::EndState::Up) {
    single::servo_left::set_ratio(12.5 / 100);
    single::servo_right::set_ratio(2.5 / 100);
    global::end_state = global::EndState::Up;
  } else {
    single::servo_left::set_ratio(2.5 / 100);
    single::servo_right::set_ratio(12.5 / 100);
    global::end_state = global::EndState::Down;
  }
}

auto toggle_servo() -> void {
  if (global::end_state == global::EndState::Up) {
    set_end_state<global::EndState::Down>();
  } else {
    set_end_state<global::EndState::Up>();
  }
}

auto apply_motion() -> void {
  if (global::watchdog_tick > 100) {
    global::rotation = 0;
    global::v = 0;
    global::state = global::State::Finding;

    single::motion.move({0., global::v}, global::rotation / 50.f);
  } else if (global::watchdog_tick > 30) {
    global::v = 0;
    single::motion.move({0., global::v}, global::rotation / 50.f);
  } else {
    if (std::abs(global::v) < 0.3 && std::abs(global::rotation) < 0.1) {
      global::state = global::State::Found;
    } else {
      single::motion.move({0., global::v}, global::rotation / 50.f);
    }
  }
}

auto update_status() -> void {
  const int32_t interval[wheel_num]{single::encoder0.interval(),
                                    single::encoder1.interval(),
                                    single::encoder2.interval()};

  debug::interval[0] = interval[0];
  debug::interval[1] = interval[1];
  debug::interval[2] = interval[2];

  single::motor0.update_speed(static_cast<float>(interval[0]) /
                              encoder_count_limit);
  single::motor1.update_speed(static_cast<float>(interval[1]) /
                              encoder_count_limit);
  single::motor2.update_speed(static_cast<float>(interval[2]) /
                              encoder_count_limit);

  global::speed[0] = single::motor0.speed();
  global::speed[1] = single::motor1.speed();
  global::speed[2] = single::motor2.speed();
}

auto serial_callback(UART_HandleTypeDef *, uint16_t) -> void {
  single::remote.receive_idle<Mode::It>(global::recv, sizeof(global::recv));

  global::watchdog_tick = 0;
  global::v = 0;
  global::rotation = 0;

  const auto package = reinterpret_cast<protocol::Package *>(global::recv);
  global::package = *package;
  if (package->header_a == 0x2c && package->header_b == 0x12 &&
      package->end == 0x5b) {
    if (global::state == global::State::Finding) {
      global::state = global::State::Tracking;
    }
    global::v = package->v;
    global::rotation = package->w;
  }
}

} // namespace mission

auto entrypoint() -> void {
  single::motor0.setup<single::pin_a0, single::pin_a1, single::pwm_a>();
  single::motor1.setup<single::pin_b0, single::pin_b1, single::pwm_b>();
  single::motor2.setup<single::pin_c0, single::pin_c1, single::pwm_c>();

  single::encoder0.setup(false);
  single::encoder1.setup();
  single::encoder2.setup(false);

  single::motor0.set_pid(0.5, 0., 0.);
  single::motor1.set_pid(0.5, 0., 0.);
  single::motor2.set_pid(0.5, 0., 0.);

  auto &receive = global::recv;
  single::remote.receive_idle<Mode::It>(receive);
  single::remote.set_callback(mission::serial_callback);

  mission::set_end_state<global::EndState::Up>();

  single::servo_left::start<hal::Mode::Normal>();
  single::servo_right::start<hal::Mode::Normal>();

  single::timer5.register_activity(1000, mission::toggle_led);
  single::timer5.register_activity(5, mission::update_status);
  single::timer5.register_activity(5, []() {
    switch (global::state) {
    case global::State::Tracking: {
      mission::apply_motion();
      return;
    }
    case global::State::Finding: {
      mission::set_end_state<global::EndState::Up>();
      break;
    }
    case global::State::Get_Nearby: {
      static auto count{0};
      if (count++ > 85 && global::end_state == global::EndState::Up) {
        single::motor0.rotate_closed_loop(0.);
        single::motor1.rotate_closed_loop(0.);
        single::motor2.rotate_closed_loop(0.);
        mission::set_end_state<global::EndState::Down>();

      } else if (count > 1085) {
        global::state = global::State::Finding;
        count = 0;
      }
      return;
    }
    case global::State::Found: {
      single::motor0.rotate_closed_loop(0.8);
      single::motor1.rotate_closed_loop(0.);
      single::motor2.rotate_closed_loop(-0.8);
      global::state = global::State::Get_Nearby;
      return;
    }
    default:
      break;
    }
  });

  single::timer5.start();

  while (true) {
    time::delay(10);
    global::watchdog_tick++;
  }
}