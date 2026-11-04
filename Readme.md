## STM32F446 Drivers

Bare-metal, register-level drivers for the STM32F446 (ARM Cortex-M4), written
without ST's HAL to review and reinforce the basics of microcontroller
programming. The project was built with STM32CubeIDE.

### Blog post

- [STM32 SPI Driver Testing with Logic Analyzer](https://jarrocha.github.io/post/spi_driver_test/):
  how the SPI driver was verified with a Saleae logic analyzer, with no second
  SPI device to talk to.

### Drivers

| File | Description |
|------|-------------|
| `drivers/Inc/stm32f446xx.h` | Device header: peripheral base addresses, register maps, clock-enable macros |
| `drivers/*/stm32f446xx_gpio_driver.*` | GPIO: init/deinit, read/write pin and port, toggle, EXTI interrupt configuration and priority |
| `drivers/*/stm32f446xx_spi_driver.*` | SPI: peripheral clock, init/deinit, SSI/SSOE configuration, blocking send/receive, IRQ configuration |

### Examples (`Src/`)

| Program | What it does |
|---------|--------------|
| `001_LED_Toggle.c` | Toggle an LED from a loop |
| `002_LED_ButtonToggle.c` | Toggle an LED with the on-board button |
| `003_LED_ButtonToggle_Ext.c` | Same, with an external LED and button |
| `005_LED_ButtonToggleIRQ.c` | Button handled through an EXTI interrupt |
| `006_SPI_SendTest.c` | SPI2 master transmit test, checked with a logic analyzer |
| `007_SPI_TX_Arduino.c` | SPI2 master sending data to an Arduino acting as the slave |

### SPI2 pin mapping

| Signal | Pin | Alternate function |
|--------|-----|--------------------|
| SPI2_NSS  | PB12 | AF5 |
| SPI2_SCLK | PB13 | AF5 |
| SPI2_MISO | PB14 | AF5 |
| SPI2_MOSI | PB15 | AF5 |
