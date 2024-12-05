#include "entrypoint.hh"
#include "protocol.hh"
#include "singleton.hh"

using namespace hal;

constexpr auto wheel_num = 3;

namespace global {
float rotation{0.f};
float speed[wheel_num]{0.f};
float v{0.f};

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

auto toggle() -> void { single::led::toggle(); }

auto apply_motion() -> void {
  if (global::watchdog_tick > 100) {
    global::rotation = 0;
    global::v = 0;
  } else if (global::watchdog_tick > 30) {
    global::v = 0;
  }

  single::motion.move({0., global::v}, global::rotation / 50.f);
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

  single::timer5.register_activity(1000, mission::toggle);
  single::timer5.register_activity(5, mission::update_status);
  single::timer5.register_activity(5, mission::apply_motion);

  single::timer5.start();

  while (true) {
    time::delay(10);
    global::watchdog_tick++;
  }
}