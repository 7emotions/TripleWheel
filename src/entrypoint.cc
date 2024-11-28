#include "entrypoint.hh"

#include "base/gpio.hh"

#include "singleton.hh"
#include <concepts>

using namespace hal;

auto led = gpio::PA<1>{};

void entrypoint() {
  while (true) {
    led.toggle();
    HAL_Delay(500);
  }
}