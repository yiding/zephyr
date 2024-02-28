#include "airoc_wifi.h"
#include "airoc_whd_hal_common.h"

#include <bus_protocols/whd_bus_spi_protocol.h>
#include <bus_protocols/whd_spi.h>
#include <whd_wifi_api.h>

// From whd_bus_spi_protocol.c
struct whd_bus_priv
{
    whd_spi_config_t spi_config;
    whd_spi_t *spi_obj;
};

#define DT_DRV_COMPAT infineon_airoc_wifi
LOG_MODULE_DECLARE(infineon_airoc_wifi, CONFIG_WIFI_LOG_LEVEL);

/**
 * Disable interrupt, return the logical level of the interrupt pin prior to
 * disabling.
 */
static int whd_irq_disable(whd_driver_t whd_driver) {
  int ret;
  struct gpio_dt_spec *gpio = 
    (struct gpio_dt_spec *)whd_driver->bus_priv->spi_config.oob_config.host_oob_pin;

  ret = gpio_pin_configure_dt(gpio, GPIO_INPUT);
  __ASSERT(ret == 0, "gpio_pin_configure_dt failed, return code %d", ret);
  int old_state = gpio_pin_get_dt(gpio);
  ret = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_DISABLE);
  __ASSERT(ret == 0, "gpio_pin_interrupt_configure_dt failed, return code %d", ret);
  return old_state;
}

static void whd_irq_enable(whd_driver_t whd_driver, int old_state) {
  int ret;
	const whd_oob_config_t *oob_config = &whd_driver->bus_priv->spi_config.oob_config;
  struct gpio_dt_spec *gpio = 
    (struct gpio_dt_spec *)oob_config->host_oob_pin;
  ret = gpio_pin_configure_dt(gpio, GPIO_INPUT);
  __ASSERT(ret == 0, "gpio_pin_configure_dt failed, return code %d", ret);
  ret = gpio_pin_interrupt_configure_dt(
    gpio, 
    (oob_config->is_falling_edge == WHD_TRUE) ? GPIO_INT_EDGE_FALLING : GPIO_INT_EDGE_RISING);
  __ASSERT(ret == 0, "gpio_pin_interrupt_configure_dt failed, return code %d", ret);
  
  int state = gpio_pin_get_dt(gpio);
  int expected_event = (oob_config->is_falling_edge == WHD_TRUE) ? 0 : 1;

  // Notify only if interrupt wasn't assert before. This code assumes that if
  // the interrupt was previously asserted, then the thread has already been
  // notified.
  if (state == expected_event && state != old_state) {
    whd_thread_notify_irq(whd_driver);
  }
}

void whd_bus_spi_oob_irq_handler(const struct device *port, struct gpio_callback *cb,
                                 gpio_port_pins_t pins)
{
	struct airoc_wifi_data *data = CONTAINER_OF(cb, struct airoc_wifi_data, host_oob_pin_cb);

	/* Get OOB pin info */
	const whd_oob_config_t *oob_config = &data->whd_drv->bus_priv->spi_config.oob_config;
	const struct gpio_dt_spec *host_oob_pin = oob_config->host_oob_pin;

	/* Check OOB state is correct */
	int expected_event = (oob_config->is_falling_edge == WHD_TRUE) ? 0 : 1;

	if (!(pins & BIT(host_oob_pin->pin)) || (gpio_pin_get_dt(host_oob_pin) != expected_event)) {
		WPRINT_WHD_ERROR(("Unexpected interrupt event %d\n", expected_event));
		return;
	}

	/* Call thread notify to wake up WHD thread */
	whd_thread_notify_irq(data->whd_drv);
}

whd_result_t whd_bus_spi_transfer(whd_driver_t whd_driver, 
          const uint8_t *tx, size_t tx_length,
				  uint8_t *rx, size_t rx_length, 
          uint8_t write_fill)
{
  ARG_UNUSED(write_fill);

  int ret;
  whd_result_t whd_ret = WHD_SUCCESS;
  struct device *dev = (struct device *)whd_driver->bus_priv->spi_obj;
  const struct airoc_wifi_config *config = dev->config;

  int old_state = whd_irq_disable(whd_driver);
  if (config->bus.config.operation & SPI_HALF_DUPLEX) {
    if (tx_length > 0) {
      // TODO(yiding): it seems like in whd_bus_spi_protocol.c the code expects
      // the rx buffer to be transmitted if tx length is non-zero while tx =
      // NULL, rather than transmitting a filler byte.
      //
      // This behavior is not documented in cyhal_spi_transfer (the default spi
      // impl we are replacing)
      if (tx == NULL && rx_length >= tx_length) {
        tx = rx;
      }
      struct spi_buf tx_bufs[] = {{
        .buf = (uint8_t *)tx,
        .len = tx_length,
      }};
      struct spi_buf_set tx_buf_set = {
        .buffers = tx_bufs,
        .count = 1,
      };
      if (tx_length > 0 && tx != NULL) {
        //LOG_HEXDUMP_INF(tx, tx_length, "TX");
      }
      ret = spi_write_dt(&config->bus, &tx_buf_set);
      if (ret != 0) {
        LOG_WRN("spi_write_dt failed: %d", ret);
        whd_ret = WHD_WLAN_SDIO_ERROR;
        goto fail;
      }
    }
    if (rx_length > 0) {
      // The driver code expects the reads and writes to be simultaneous if both
      // read and write are requested.  So if writes are requested we skip the
      // first X bytes of the read buffer.
      struct spi_buf rx_bufs[] = {{
          .buf = rx != NULL ? rx + tx_length : NULL,
          .len = rx_length >= tx_length ? rx_length - tx_length : 0,
      }};
      if (rx_length >= tx_length && rx != NULL) {
        // zero the skipped bytes.
        memset(rx, 0, tx_length);
      }
      struct spi_buf_set rx_buf_set = {
        .buffers = rx_bufs,
        .count = 1,
      };
      if (rx_length > 0 && rx != NULL) {
        //LOG_HEXDUMP_INF(rx, rx_length, "RX");
      }
      ret = spi_read_dt(&config->bus, &rx_buf_set);
      if (ret != 0) {
        LOG_WRN("spi_read_dt failed: %d", ret);
        whd_ret = WHD_WLAN_SDIO_ERROR;
        goto fail;
      }
    }
  } else {
    __ASSERT(false, "not implemented");
  }
fail:
  whd_irq_enable(whd_driver, old_state);
  return whd_ret;
}

whd_result_t whd_bus_spi_irq_register(whd_driver_t whd_driver) {
  int ret;
  struct device *dev = (struct device *)whd_driver->bus_priv->spi_obj;
  struct airoc_wifi_data *data = dev->data;
  struct gpio_dt_spec *gpio = 
    (struct gpio_dt_spec *)whd_driver->bus_priv->spi_config.oob_config.host_oob_pin;

  gpio_init_callback(&data->host_oob_pin_cb, whd_bus_spi_oob_irq_handler, BIT(gpio->pin));
  ret = gpio_add_callback_dt(gpio, &data->host_oob_pin_cb);
  __ASSERT(ret == 0, "gpio_add_callback_dt failed, return code %d", ret);

  return WHD_SUCCESS;
}

whd_result_t whd_bus_spi_irq_enable(whd_driver_t whd_driver, whd_bool_t enable) {
	const whd_oob_config_t *oob_config = &whd_driver->bus_priv->spi_config.oob_config;
  if (enable == WHD_TRUE) {
    whd_irq_enable(whd_driver, oob_config->is_falling_edge == WHD_TRUE ? 1 : 0);
  } else {
    whd_irq_disable(whd_driver);
  }
  return WHD_SUCCESS;
}

static whd_init_config_t init_config_default = {
	.thread_stack_size = CY_WIFI_THREAD_STACK_SIZE,
	.thread_stack_start = NULL,
	.thread_priority = (uint32_t)CY_WIFI_THREAD_PRIORITY,
	.country = CY_WIFI_COUNTRY
};

int airoc_wifi_init_primary(const struct device *dev, whd_interface_t *interface,
			    whd_netif_funcs_t *netif_funcs, whd_buffer_funcs_t *buffer_if) {
  int ret;
  cy_rslt_t whd_ret;
  struct airoc_wifi_data *data = dev->data;
  const struct airoc_wifi_config *config = dev->config;

  ret = airoc_wifi_power_on(dev);
  __ASSERT(ret == 0, "airoc_wifi_power_on failed, return code %d", ret);

  // TODO(yiding): translate the return codes and return as appropriate.
	whd_ret = whd_init(&data->whd_drv, &init_config_default, &resource_ops, buffer_if, netif_funcs);
  __ASSERT(whd_ret == CY_RSLT_SUCCESS, "whd_init failed, return code %d", whd_ret);

  whd_ret = whd_bus_spi_attach(data->whd_drv, (whd_spi_config_t *)&config->spi_config,
			       (whd_spi_t *)dev);
  __ASSERT(whd_ret == CY_RSLT_SUCCESS, "whd_bus_spi_attach failed, return code %d", whd_ret);
  LOG_INF("whd_bus_spi_attach success");

  whd_ret = whd_wifi_on(data->whd_drv, interface);
  __ASSERT(whd_ret == CY_RSLT_SUCCESS, "whd_wifi_on failed, return code %d", whd_ret);
  LOG_INF("whd_wifi_on success");

  return 0;
}