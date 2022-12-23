/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT microchip_at86rf215

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

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
	struct gpio_dt_spec reset_gpio;
};

static const struct at86rf215_config dev_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),
	.reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios)
};

static struct at86rf215_data dev_data;

void at86rf215_rst(const struct device *dev)
{
	const struct at86rf215_config *conf = dev->config;

	/* Ensure control lines have correct levels. */
	gpio_pin_set_dt(&conf->reset_gpio, 0);

	/* Wait typical time of timer TR1. */
	k_busy_wait(330);

	gpio_pin_set_dt(&conf->reset_gpio, 1);
	k_busy_wait(10);
	gpio_pin_set_dt(&conf->reset_gpio, 0);
}

static int at86rf215_init(const struct device *dev)
{
	const struct at86rf215_config *config = dev->config;
	
	LOG_DBG("Initializing at86rf215");

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	at86rf215_rst(dev);

	// Woj's code goes here :) 

	LOG_DBG("at86rf215 Initialized");
	
	return 0;
}

DEVICE_DT_INST_DEFINE(0, at86rf215_init, NULL, &dev_data,
		      &dev_config, POST_KERNEL, 90,
		      NULL);