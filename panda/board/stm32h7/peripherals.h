void gpio_usb_init(void) {
  // A11,A12: USB:
  set_gpio_alternate(GPIOA, 11, GPIO_AF10_OTG1_FS);
  set_gpio_alternate(GPIOA, 12, GPIO_AF10_OTG1_FS);
  GPIOA->OSPEEDR = GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12;
}

void gpio_usart2_init(void) {
  // A2,A3: USART 2 for debugging
  set_gpio_alternate(GPIOA, 2, GPIO_AF7_USART2);
  set_gpio_alternate(GPIOA, 3, GPIO_AF7_USART2);
}

void gpio_uart7_init(void) {
  // E7,E8: UART 7 for debugging
  set_gpio_alternate(GPIOE, 7, GPIO_AF7_UART7);
  set_gpio_alternate(GPIOE, 8, GPIO_AF7_UART7);
}

// Common GPIO initialization
void common_init_gpio(void) {
  /// E2,E3,E4: RGB LED
  set_gpio_pullup(GPIOE, 2, PULL_NONE);
  set_gpio_mode(GPIOE, 2, MODE_OUTPUT);
  set_gpio_output_type(GPIOE, 2, OUTPUT_TYPE_OPEN_DRAIN);

  set_gpio_pullup(GPIOE, 3, PULL_NONE);
  set_gpio_mode(GPIOE, 3, MODE_OUTPUT);
  set_gpio_output_type(GPIOE, 3, OUTPUT_TYPE_OPEN_DRAIN);

  set_gpio_pullup(GPIOE, 4, PULL_NONE);
  set_gpio_mode(GPIOE, 4, MODE_OUTPUT);
  set_gpio_output_type(GPIOE, 4, OUTPUT_TYPE_OPEN_DRAIN);

  // F7,F8,F9,F10: BOARD ID
  set_gpio_pullup(GPIOF, 7, PULL_NONE);
  set_gpio_mode(GPIOF, 7, MODE_INPUT);

  set_gpio_pullup(GPIOF, 8, PULL_NONE);
  set_gpio_mode(GPIOF, 8, MODE_INPUT);

  set_gpio_pullup(GPIOF, 9, PULL_NONE);
  set_gpio_mode(GPIOF, 9, MODE_INPUT);

  set_gpio_pullup(GPIOF, 10, PULL_NONE);
  set_gpio_mode(GPIOF, 10, MODE_INPUT);

  //C4,A1: OBD_SBU1, OBD_SBU2
  set_gpio_pullup(GPIOC, 4, PULL_NONE);
  set_gpio_mode(GPIOC, 4, MODE_ANALOG);

  set_gpio_pullup(GPIOA, 1, PULL_NONE);
  set_gpio_mode(GPIOA, 1, MODE_ANALOG);

  //F11: VOLT_S
  set_gpio_pullup(GPIOF, 11, PULL_NONE);
  set_gpio_mode(GPIOF, 11, MODE_ANALOG);

  gpio_usb_init();

  // B8,B9: FDCAN1
  set_gpio_pullup(GPIOB, 8, PULL_NONE);
  set_gpio_alternate(GPIOB, 8, GPIO_AF9_FDCAN1);

  set_gpio_pullup(GPIOB, 9, PULL_NONE);
  set_gpio_alternate(GPIOB, 9, GPIO_AF9_FDCAN1);

  // B5,B6 (mplex to B12,B13): FDCAN2
  set_gpio_pullup(GPIOB, 12, PULL_NONE);
  set_gpio_pullup(GPIOB, 13, PULL_NONE);

  set_gpio_pullup(GPIOB, 5, PULL_NONE);
  set_gpio_alternate(GPIOB, 5, GPIO_AF9_FDCAN2);

  set_gpio_pullup(GPIOB, 6, PULL_NONE);
  set_gpio_alternate(GPIOB, 6, GPIO_AF9_FDCAN2);

  // G9,G10: FDCAN3
  set_gpio_pullup(GPIOG, 9, PULL_NONE);
  set_gpio_alternate(GPIOG, 9, GPIO_AF2_FDCAN3);

  set_gpio_pullup(GPIOG, 10, PULL_NONE);
  set_gpio_alternate(GPIOG, 10, GPIO_AF2_FDCAN3);
}

void flasher_peripherals_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_USB1OTGHSEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
}

// Peripheral initialization
void peripherals_init(void) {
  // enable GPIO(A,B,C,D,E,F,G,H)
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN;
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;

  RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;  // SPI
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;  // SPI DMA
  RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;  // main counter
  RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;  // interrupt timer
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;  // clock source timer
  RCC->APB1LENR |= RCC_APB1LENR_TIM12EN;  // slow loop

  RCC->APB1HENR |= RCC_APB1HENR_FDCANEN; // FDCAN core enable
  RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN; // Enable ADC clocks

  RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;

  // HS USB enable, also LP is needed for CSleep state(__WFI())
  RCC->AHB1ENR |= RCC_AHB1ENR_USB1OTGHSEN;
  RCC->AHB1LPENR |= RCC_AHB1LPENR_USB1OTGHSLPEN;
  RCC->AHB1LPENR &= ~(RCC_AHB1LPENR_USB1OTGHSULPILPEN);
}

void enable_interrupt_timer(void) {
  register_set_bits(&(RCC->APB1LENR), RCC_APB1LENR_TIM6EN); // Enable interrupt timer peripheral
}
