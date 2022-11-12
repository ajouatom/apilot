// ///////////////////// //
// Red Panda chiplet + Harness //
// ///////////////////// //

// Most hardware functionality is similar to red panda

void red_chiplet_enable_can_transceiver(uint8_t transceiver, bool enabled) {
  switch (transceiver) {
    case 1U:
      set_gpio_output(GPIOG, 11, !enabled);
      break;
    case 2U:
      set_gpio_output(GPIOB, 10, !enabled);
      break;
    case 3U:
      set_gpio_output(GPIOD, 7, !enabled);
      break;
    case 4U:
      set_gpio_output(GPIOB, 11, !enabled);
      break;
    default:
      break;
  }
}

void red_chiplet_enable_can_transceivers(bool enabled) {
  uint8_t main_bus = (car_harness_status == HARNESS_STATUS_FLIPPED) ? 3U : 1U;
  for (uint8_t i=1U; i<=4U; i++) {
    // Leave main CAN always on for CAN-based ignition detection
    if (i == main_bus) {
      red_chiplet_enable_can_transceiver(i, true);
    } else {
      red_chiplet_enable_can_transceiver(i, enabled);
    }
  }
}

void red_chiplet_set_usb_load_switch(bool enabled) {
  set_gpio_output(GPIOD, 3, enabled);
}

void red_chiplet_init(void) {
  common_init_gpio();

  //A8, A9 : OBD_SBU1_RELAY, OBD_SBU2_RELAY
  set_gpio_output_type(GPIOA, 8, OUTPUT_TYPE_OPEN_DRAIN);
  set_gpio_pullup(GPIOA, 8, PULL_NONE);
  set_gpio_mode(GPIOA, 8, MODE_OUTPUT);
  set_gpio_output(GPIOA, 8, 1);

  set_gpio_output_type(GPIOA, 9, OUTPUT_TYPE_OPEN_DRAIN);
  set_gpio_pullup(GPIOA, 9, PULL_NONE);
  set_gpio_mode(GPIOA, 9, MODE_OUTPUT);
  set_gpio_output(GPIOA, 9, 1);

  // G11,B10,D7,B11: transceiver enable
  set_gpio_pullup(GPIOG, 11, PULL_NONE);
  set_gpio_mode(GPIOG, 11, MODE_OUTPUT);

  set_gpio_pullup(GPIOB, 10, PULL_NONE);
  set_gpio_mode(GPIOB, 10, MODE_OUTPUT);

  set_gpio_pullup(GPIOD, 7, PULL_NONE);
  set_gpio_mode(GPIOD, 7, MODE_OUTPUT);

  set_gpio_pullup(GPIOB, 11, PULL_NONE);
  set_gpio_mode(GPIOB, 11, MODE_OUTPUT);

  // D3: usb load switch
  set_gpio_pullup(GPIOD, 3, PULL_NONE);
  set_gpio_mode(GPIOD, 3, MODE_OUTPUT);

  //B0: 5VOUT_S
  set_gpio_pullup(GPIOB, 0, PULL_NONE);
  set_gpio_mode(GPIOB, 0, MODE_ANALOG);

  // Turn on USB load switch.
  red_chiplet_set_usb_load_switch(true);

  // Initialize harness
  harness_init();

  // Initialize RTC
  rtc_init();

  // Enable CAN transceivers
  red_chiplet_enable_can_transceivers(true);

  // Disable LEDs
  red_set_led(LED_RED, false);
  red_set_led(LED_GREEN, false);
  red_set_led(LED_BLUE, false);

  // Set normal CAN mode
  red_set_can_mode(CAN_MODE_NORMAL);

  // flip CAN0 and CAN2 if we are flipped
  if (car_harness_status == HARNESS_STATUS_FLIPPED) {
    can_flip_buses(0, 2);
  }
}

const harness_configuration red_chiplet_harness_config = {
  .has_harness = true,
  .GPIO_SBU1 = GPIOC,
  .GPIO_SBU2 = GPIOA,
  .GPIO_relay_SBU1 = GPIOA,
  .GPIO_relay_SBU2 = GPIOA,
  .pin_SBU1 = 4,
  .pin_SBU2 = 1,
  .pin_relay_SBU1 = 8,
  .pin_relay_SBU2 = 9,
  .adc_channel_SBU1 = 4, // ADC12_INP4
  .adc_channel_SBU2 = 17 // ADC1_INP17
};
