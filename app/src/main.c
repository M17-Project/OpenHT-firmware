
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

// devicetree node alias
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void main(void)
{
  int ret;

	if (!device_is_ready(led1.port)) {
		return;
	}

  ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

  while(1) 
  {
    gpio_pin_toggle_dt(&led1);

    k_msleep(SLEEP_TIME_MS);
  }
}
