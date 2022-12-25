/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT microchip_at86rf215

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(microchip_at86rf215, 4);

#define DT_DRV_COMPAT microchip_at86rf215

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(microchip_at86rf215) <= 1,
             "Multiple AT86RF2125 instances in DT");

struct at86rf215_data {
  float frequency_mhz;
};

struct at86rf215_config {
  struct spi_dt_spec bus;
  struct gpio_dt_spec nrst_gpio;
};

static const struct at86rf215_config dev_config = {
    .bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),
    .nrst_gpio = GPIO_DT_SPEC_INST_GET(0, nrst_gpios)};

static struct at86rf215_data dev_data;

static int at86rf215_write(const struct device *dev, const uint16_t addr,
                           const uint8_t val) {
  const struct at86rf215_config *config = dev->config;

  // write enable
  uint8_t tx[3] = {(1 << 7) | (addr >> 8), addr & 0xFF, val};

  // Setup buffer description
  struct spi_buf tx_buffer_info[1] = {{.buf = tx, .len = ARRAY_SIZE(tx)}};

  // Add buffer to buffer set
  const struct spi_buf_set tx_buff_set = {tx_buffer_info, 1};

  return spi_transceive_dt(&config->bus, &tx_buff_set, NULL);
}

static uint8_t at86rf215_read(const struct device *dev, const uint16_t addr) {
  const struct at86rf215_config *config = dev->config;

  // TX buffer setup

  // Setup read command
  uint8_t tx[3] = {addr >> 8, addr & 0xFF, 0xFF};

  // Setup buffer description
  struct spi_buf tx_buffer_info[1] = {{.buf = tx, .len = ARRAY_SIZE(tx)}};

  // Add buffer to buffer set
  const struct spi_buf_set tx_buff_set = {tx_buffer_info, 1};

  // RX buffer setup

  uint8_t rx[3];

  // Setup buffer description
  struct spi_buf rx_buffer_info[1] = {{.buf = rx, .len = ARRAY_SIZE(rx)}};

  // Add buffer to buffer set
  const struct spi_buf_set rx_buff_set = {rx_buffer_info, 1};

  spi_transceive_dt(&config->bus, &tx_buff_set, &rx_buff_set);

  return rx[2];
}

static void at86rf215_reset(const struct device *dev) {
  const struct at86rf215_config *config = dev->config;

  gpio_pin_set_dt(&config->nrst_gpio, 0);
  k_busy_wait(10000);
  gpio_pin_set_dt(&config->nrst_gpio, 1);
  k_busy_wait(10000);
}

static uint8_t at86rf215_read_id(const struct device *dev) {
  return at86rf215_read(dev, 0x000D);
}

static int at86rf215_vco0_set(const struct device *dev, const uint32_t freq) {
  const struct at86rf215_config *config = dev->config;

  if (freq >= 389500000 && freq <= 510000000) // range check
  {
    uint32_t val = round((freq - 377000000) / (203125.0 / 2048.0));

    // use the block write starting at 0x0105
    // Fine Resolution Channel Scheme CNM.CM=1, 389.5-510.0MHz with 99.182Hz
    // channel stepping
    uint8_t regs[6] = {0x01 | (1 << 7),   0x05,
                       (val >> 8) & 0xFF, (val >> 16) & 0xFF,
                       (val >> 0) & 0xFF, 1 << 6};

    // Setup buffer description
    struct spi_buf tx_buffer_info[1] = {{.buf = regs, .len = ARRAY_SIZE(regs)}};

    // Add buffer to buffer set
    const struct spi_buf_set tx_buff_set = {tx_buffer_info, 1};

    spi_transceive_dt(&config->bus, &tx_buff_set, NULL);

    return 0;
  }
  else if (freq >= 779000000 && freq <= 1020000000) // range check
  {
    uint32_t val = round((freq - 754000000) / (203125.0 / 1024.0));

    // use the block write starting at 0x0105
    // Fine Resolution Channel Scheme CNM.CM=2, 779-1020MHz with 198.364Hz
    // channel stepping
    uint8_t regs[6] = {0x01 | (1 << 7),   0x05,
                       (val >> 8) & 0xFF, (val >> 16) & 0xFF,
                       (val >> 0) & 0xFF, 2 << 6};

    // Setup buffer description
    struct spi_buf tx_buffer_info[1] = {{.buf = regs, .len = ARRAY_SIZE(regs)}};

    // Add buffer to buffer set
    const struct spi_buf_set tx_buff_set = {tx_buffer_info, 1};

    spi_transceive_dt(&config->bus, &tx_buff_set, NULL);

    return 0;
  }

  return 1;
}

static int at86rf215_init(const struct device *dev) {
  const struct at86rf215_config *config = dev->config;

  LOG_DBG("Initializing at86rf215");

  if (!spi_is_ready_dt(&config->bus)) {
    LOG_ERR("SPI device not ready");
    return -ENODEV;
  }

  at86rf215_reset(dev);

  uint8_t dev_id = at86rf215_read_id(dev);

  if (dev_id == 0x34) // correct chip && SPI comms OK?
  {
    // IQIFC1.CHPM=1; IQ mode for both transceivers
    at86rf215_write(dev, 0x000B, 1 << 4);
    // IQIFC1.CHPM=0
    // at86rf215_write(dev, 0x000B, 0);

    // BBC0_PC.CTX=1
    at86rf215_write(dev, 0x0301, 0b11010100);

    // RF09_TXCUTC=0; PA ramp and lowpass set to the minimum (80kHz)
    at86rf215_write(dev, 0x0112, 0);

    // sample rate 400kHz, f_cut at 1/4 f_s
    at86rf215_write(dev, 0x0113, 0xA);

    // set the sub-ghz VCO frequency to 435 megs
    at86rf215_vco0_set(dev, 435000000 * (1.0 + 0.9e-6));

    // power output to max - default setting
    // at86rf215_write(dev, 0x0114, 0x7F);

    // frame length to max (2047)
    at86rf215_write(dev, 0x0306, 0xFF);
    at86rf215_write(dev, 0x0307, 0x07);

    // set mode to TXPREP; CMD=TXPREP
    at86rf215_write(dev, 0x0103, 0x03);

    // set I sample source to internal DAC, set to max
    at86rf215_write(dev, 0x0127, 0x7F | (1 << 7));
    // set Q sample source to internal DAC, set to 0
    at86rf215_write(dev, 0x0128, 0x3F | (1 << 7));

    // issue TX command
    at86rf215_write(dev, 0x0103, 0x04);

    LOG_DBG("at86rf215 Initialized");
    return 0;
  }
  else
  {
    LOG_ERR("Unexpected ID from at86rf215: 0x%02X", dev_id);
    return -1;
  }
}

DEVICE_DT_INST_DEFINE(0, at86rf215_init, NULL, &dev_data, &dev_config,
                      POST_KERNEL, 90, NULL);
