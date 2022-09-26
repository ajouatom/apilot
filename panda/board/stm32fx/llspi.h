// IRQs: DMA2_Stream2, DMA2_Stream3, EXTI4

void spi_init(void);
int spi_cb_rx(uint8_t *data, int len, uint8_t *data_out);

// end API

#define SPI_BUF_SIZE 256
uint8_t spi_buf[SPI_BUF_SIZE];
int spi_buf_count = 0;
int spi_total_count = 0;

void spi_tx_dma(void *addr, int len) {
  // disable DMA
  register_clear_bits(&(SPI1->CR2), SPI_CR2_TXDMAEN);
  register_clear_bits(&(DMA2_Stream3->CR), DMA_SxCR_EN);

  // DMA2, stream 3, channel 3
  register_set(&(DMA2_Stream3->M0AR), (uint32_t)addr, 0xFFFFFFFFU);
  DMA2_Stream3->NDTR = len;
  register_set(&(DMA2_Stream3->PAR), (uint32_t)&(SPI1->DR), 0xFFFFFFFFU);

  // channel3, increment memory, memory -> periph, enable
  register_set(&(DMA2_Stream3->CR), (DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_EN), 0x1E077EFEU);
  delay(0);
  register_set_bits(&(DMA2_Stream3->CR), DMA_SxCR_TCIE);

  register_set_bits(&(SPI1->CR2), SPI_CR2_TXDMAEN);

  // signal data is ready by driving low
  // esp must be configured as input by this point
  set_gpio_output(GPIOB, 0, 0);
}

void spi_rx_dma(void *addr, int len) {
  // disable DMA
  register_clear_bits(&(SPI1->CR2), SPI_CR2_RXDMAEN);
  register_clear_bits(&(DMA2_Stream2->CR), DMA_SxCR_EN);

  // drain the bus
  volatile uint8_t dat = SPI1->DR;
  (void)dat;

  // DMA2, stream 2, channel 3
  register_set(&(DMA2_Stream2->M0AR), (uint32_t)addr, 0xFFFFFFFFU);
  DMA2_Stream2->NDTR = len;
  register_set(&(DMA2_Stream2->PAR), (uint32_t)&(SPI1->DR), 0xFFFFFFFFU);

  // channel3, increment memory, periph -> memory, enable
  register_set(&(DMA2_Stream2->CR), (DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC | DMA_SxCR_EN), 0x1E077EFEU);
  delay(0);
  register_set_bits(&(DMA2_Stream2->CR), DMA_SxCR_TCIE);

  register_set_bits(&(SPI1->CR2), SPI_CR2_RXDMAEN);
}

// ***************************** SPI IRQs *****************************
// can't go on the stack cause it's DMAed
uint8_t spi_tx_buf[0x44];

// SPI RX
void DMA2_Stream2_IRQ_Handler(void) {
  int *resp_len = (int*)spi_tx_buf;
  (void)memset(spi_tx_buf, 0xaa, 0x44);
  *resp_len = spi_cb_rx(spi_buf, 0x14, spi_tx_buf+4);
  #ifdef DEBUG_SPI
    puts("SPI write: ");
    puth(*resp_len);
    puts("\n");
  #endif
  spi_tx_dma(spi_tx_buf, *resp_len + 4);

  // ack
  DMA2->LIFCR = DMA_LIFCR_CTCIF2;
}

// SPI TX
void DMA2_Stream3_IRQ_Handler(void) {
  #ifdef DEBUG_SPI
    puts("SPI handshake\n");
  #endif

  // reset handshake back to pull up
  set_gpio_mode(GPIOB, 0, MODE_INPUT);
  set_gpio_pullup(GPIOB, 0, PULL_UP);

  // ack
  DMA2->LIFCR = DMA_LIFCR_CTCIF3;
}

void EXTI4_IRQ_Handler(void) {
  volatile unsigned int pr = EXTI->PR & (1U << 4);
  #ifdef DEBUG_SPI
    puts("exti4\n");
  #endif
  // SPI CS falling
  if ((pr & (1U << 4)) != 0U) {
    spi_total_count = 0;
    spi_rx_dma(spi_buf, 0x14);
  }
  EXTI->PR = pr;
}

// ***************************** SPI init *****************************
void spi_init(void) {
  // Max SPI clock the ESP can produce is 80MHz. At buffer size of 256 bytes, that's a max of about 40k buffers per second
  REGISTER_INTERRUPT(DMA2_Stream2_IRQn, DMA2_Stream2_IRQ_Handler, 50000U, FAULT_INTERRUPT_RATE_SPI_DMA)
  REGISTER_INTERRUPT(DMA2_Stream3_IRQn, DMA2_Stream3_IRQ_Handler, 50000U, FAULT_INTERRUPT_RATE_SPI_DMA)
  REGISTER_INTERRUPT(EXTI4_IRQn, EXTI4_IRQ_Handler, 50000U, FAULT_INTERRUPT_RATE_SPI_CS) // TODO: Figure out if this is a reasonable limit

  //puts("SPI init\n");
  register_set(&(SPI1->CR1), SPI_CR1_SPE, 0xFFFFU);

  // enable SPI interrupts
  //SPI1->CR2 = SPI_CR2_RXNEIE | SPI_CR2_ERRIE | SPI_CR2_TXEIE;
  register_set(&(SPI1->CR2), SPI_CR2_RXNEIE, 0xF7U);

  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  //NVIC_EnableIRQ(SPI1_IRQn);

  // reset handshake back to pull up
  set_gpio_mode(GPIOB, 0, MODE_INPUT);
  set_gpio_pullup(GPIOB, 0, PULL_UP);

  // setup interrupt on falling edge of SPI enable (on PA4)
  register_set(&(SYSCFG->EXTICR[2]), SYSCFG_EXTICR2_EXTI4_PA, 0xFFFFU);
  register_set_bits(&(EXTI->IMR), (1U << 4));
  register_set_bits(&(EXTI->FTSR), (1U << 4));
  NVIC_EnableIRQ(EXTI4_IRQn);
}
