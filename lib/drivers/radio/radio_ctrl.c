#define DT_DRV_COMPAT da_sx1262_radio

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/radio_ctrl.h>
#include "zephyr/drivers/sx126x/sx126x_hal.h"
#include "zephyr/drivers/ralf/ralf_sx126x.h"

LOG_MODULE_REGISTER(radio_ctrl, CONFIG_RADIO_CTRL_LOG_LEVEL);

#define RADIO_CTRL_MAX_MSG_SIZE      (255)
#define RADIO_CTRL_THREAD_STACK_SIZE (2 * 1024)
#define RADIO_CTRL_THREAD_PRIORITY   (K_PRIO_COOP(7))
#define RADIO_CTRL_TIMEOUT_MS        (1000)

struct radio_ctrl_msg {
    uint16_t size;
    uint8_t data[RADIO_CTRL_MAX_MSG_SIZE];
    struct msg_stats stats;
};

struct radio_ctrl_irq_stats {
    volatile uint32_t rx_done;
    volatile uint32_t tx_done;
    volatile uint32_t timeout;
    volatile uint32_t hdr_error;
    volatile uint32_t crc_error;
    volatile uint32_t cad_done;
    volatile uint32_t cad_ok;
    volatile uint32_t none;
};

struct radio_ctrl_stats {
    uint32_t tx_request;
    uint32_t tx_success;
    uint32_t tx_timeout;
    uint32_t rx_request;
    uint32_t rx_success;
    uint32_t rx_timeout;
    struct radio_ctrl_irq_stats irq;
};

struct radio_ctrl_config {
    const struct spi_dt_spec spi;
    struct gpio_dt_spec nreset;
    struct gpio_dt_spec busy;
    struct gpio_dt_spec irq;
    struct gpio_dt_spec ant_sw;
};

struct radio_ctrl_data {
    struct k_work irq_work;
    struct k_mutex mutex;
    struct k_sem sem_tx_done;
    struct k_sem sem_irq;
    struct k_sem sem_cad_done;
    struct gpio_callback irq_cb_data;
    const struct device *dev;
    bool is_cad_enabled;
    ralf_t radio;
    ralf_params_lora_t rx_params;
    ralf_params_lora_t tx_params;
    ralf_params_lora_cad_t cad_params;
    struct radio_ctrl_stats stats;
    sx126x_hal_context_t sx126x_hal_ctx;
    bool is_configured;
    bool is_ralf_initialized;
    bool is_initialized;
};

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command,
                                     const uint16_t command_length, const uint8_t *data,
                                     const uint16_t data_length)
{
    sx126x_hal_context_t *ctx = (sx126x_hal_context_t *)context;
    const struct spi_dt_spec *spi = (const struct spi_dt_spec *)ctx->spi;
    const struct gpio_dt_spec *busy = (const struct gpio_dt_spec *)ctx->busy_pin;
    struct spi_buf tx_bufs[2];
    struct spi_buf_set tx;
    int busy_state;
    int32_t busy_start = k_uptime_get_32();

    LOG_DBG("BUSY pin state: %d", gpio_pin_get_dt(busy));
    do {
        busy_state = gpio_pin_get_dt(busy);
        if (busy_state == 0) {
            break;
        }
        k_msleep(1);
    } while ((k_uptime_get_32() - busy_start) < ctx->timeout);
    if ((k_uptime_get_32() - busy_start) >= ctx->timeout) {
        return SX126X_HAL_STATUS_ERROR;
    }

    tx_bufs[0].buf = (void *)command;
    tx_bufs[0].len = command_length;
    tx_bufs[1].buf = (void *)data;
    tx_bufs[1].len = data_length;
    tx.buffers = tx_bufs;
    tx.count = (data_length > 0) ? 2 : 1;

    int ret = spi_write_dt(spi, &tx);
    return (ret == 0) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length, uint8_t *data,
                                    const uint16_t data_length)
{
    sx126x_hal_context_t *ctx = (sx126x_hal_context_t *)context;
    const struct spi_dt_spec *spi = (const struct spi_dt_spec *)ctx->spi;
    const struct gpio_dt_spec *busy = (const struct gpio_dt_spec *)ctx->busy_pin;
    uint8_t dummy_tx[data_length];
    memset(dummy_tx, 0x00, data_length);
    uint8_t dummy_rx[command_length];
    struct spi_buf tx_bufs[2];
    struct spi_buf rx_bufs[2];
    struct spi_buf_set tx, rx;
    int busy_state;
    int32_t busy_start = k_uptime_get_32();

    LOG_DBG("BUSY pin state: %d", gpio_pin_get_dt(busy));
    do {
        busy_state = gpio_pin_get_dt(busy);
        if (busy_state == 0) {
            break;
        }
        k_msleep(1);
    } while ((k_uptime_get_32() - busy_start) < ctx->timeout);
    if ((k_uptime_get_32() - busy_start) >= ctx->timeout) {
        return SX126X_HAL_STATUS_ERROR;
    }

    // TX: send command, then dummy bytes
    tx_bufs[0].buf = (void *)command;
    tx_bufs[0].len = command_length;
    tx_bufs[1].buf = dummy_tx;
    tx_bufs[1].len = data_length;
    tx.buffers = tx_bufs;
    tx.count = 2;

    // RX: ignore command response, then read data
    rx_bufs[0].buf = dummy_rx;
    rx_bufs[0].len = command_length;
    rx_bufs[1].buf = data;
    rx_bufs[1].len = data_length;
    rx.buffers = rx_bufs;
    rx.count = 2;

    int ret = spi_transceive_dt(spi, &tx, &rx);
    return (ret == 0) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
    sx126x_hal_context_t *ctx = (sx126x_hal_context_t *)context;
    const struct gpio_dt_spec *nreset = (const struct gpio_dt_spec *)ctx->nreset_pin;
    int ret;
    ret = gpio_pin_set_dt(nreset, 0);
    if (ret < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    k_msleep(2);
    ret = gpio_pin_set_dt(nreset, 1);
    if (ret < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    k_msleep(2);
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context)
{
    sx126x_hal_context_t *ctx = (sx126x_hal_context_t *)context;
    const struct gpio_dt_spec *nreset = (const struct gpio_dt_spec *)ctx->nreset_pin;
    int ret;
    ret = gpio_pin_set_dt(nreset, 0);
    if (ret < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    k_msleep(2);
    ret = gpio_pin_set_dt(nreset, 1);
    if (ret < 0) {
        return SX126X_HAL_STATUS_ERROR;
    }
    k_msleep(2);
    return SX126X_HAL_STATUS_OK;
}

static int radio_ctrl_impl_set_type(const struct device *dev, enum radio_type type)
{
    struct radio_ctrl_data *data = dev->data;
    struct radio_ctrl_config *cfg = dev->config;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    switch (type) {
    case RADIO_TYPE_SX126X: {
        data->sx126x_hal_ctx.spi = (void *)&cfg->spi;
        data->sx126x_hal_ctx.busy_pin = &cfg->busy;
        data->sx126x_hal_ctx.nreset_pin = &cfg->nreset;
        data->sx126x_hal_ctx.timeout = RADIO_CTRL_TIMEOUT_MS;

        ralf_t radio = RALF_SX126X_INSTANTIATE(&data->sx126x_hal_ctx);
        data->radio = radio;
        data->is_ralf_initialized = true;
        break;
    }
    case RADIO_TYPE_SX127X: {
        ret = -ENOTSUP;
        break;
    }
    case RADIO_TYPE_SX128X: {
        ret = -ENOTSUP;
        break;
    }
    case RADIO_TYPE_LR1110: {
        ret = -ENOTSUP;
        break;
    }
    default:
        ret = -EINVAL;
        break;
    }

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_config(const struct device *dev, const ralf_params_lora_t *rx_params,
                                  const ralf_params_lora_t *tx_params,
                                  const ralf_params_lora_cad_t *cad_params)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    if ((!rx_params) || (!tx_params) || (!cad_params)) {
        return -EINVAL;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);

    data->rx_params = *rx_params;
    data->tx_params = *tx_params;
    data->cad_params = *cad_params;
    data->is_configured = true;

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_set_antenna_switch(const struct device *dev, bool tx_enable)
{
    struct radio_ctrl_config *cfg = dev->config;

    if (!cfg->ant_sw.port) {
        return -ENOTSUP;
    }

    return gpio_pin_set_dt(&cfg->ant_sw, tx_enable ? 1 : 0);
}

static int radio_ctrl_impl_get_lora_rx_params(const struct device *dev, ralf_params_lora_t *params)
{
    struct radio_ctrl_data *data = (struct radio_ctrl_data *)dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    *params = data->rx_params;

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_set_lora_rx_params(const struct device *dev,
                                              const ralf_params_lora_t *params)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    data->rx_params = *params;

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_get_lora_tx_params(const struct device *dev, ralf_params_lora_t *params)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    *params = data->tx_params;

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_set_lora_tx_params(const struct device *dev,
                                              const ralf_params_lora_t *params)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    data->tx_params = *params;

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_get_lora_cad_params(const struct device *dev,
                                               ralf_params_lora_cad_t *params)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    *params = data->cad_params;

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_set_lora_cad_params(const struct device *dev,
                                               const ralf_params_lora_cad_t *params)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    data->cad_params = *params;

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_listen(const struct device *dev, uint32_t timeout)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    // TODO implement listen logic here using data->modem_radio and data->rx_params

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_cad(const struct device *dev, uint32_t timeout)
{
    struct radio_ctrl_data *data = dev->data;
    int ret = 0;

    k_mutex_lock(&data->mutex, K_FOREVER);

    // TODO implement CAD logic here using data->modem_radio and data->cad_params

    k_mutex_unlock(&data->mutex);

    return ret;
}

static int radio_ctrl_impl_transmit(const struct device *dev, const uint8_t *data, size_t size)
{
    struct radio_ctrl_data *ctrl_data = dev->data;
    int ret = 0;
    ral_status_t ral_status;
    uint32_t timeout;
    bool switch_to_standby = false;
    ralf_params_lora_t lora_params = {0};

    k_mutex_lock(&ctrl_data->mutex, K_FOREVER);

    do
    {
        timeout = ral_get_lora_time_on_air_in_ms(ral_from_ralf(&ctrl_data->radio),
                                                 &ctrl_data->tx_params.pkt_params,
                                                 &ctrl_data->tx_params.mod_params);
        timeout += 100; // add 100 ms for processing delay

        memcpy(&lora_params, &ctrl_data->tx_params, sizeof(ralf_params_lora_t));
        lora_params.pkt_params.pld_len_in_bytes = size;

        // set the radio to standby
        ral_status = ral_set_standby(ral_from_ralf(&ctrl_data->radio), RAL_STANDBY_CFG_RC);
        if (RAL_STATUS_OK != ral_status)
        {
            ret = -1;
            LOG_ERR("Failed to set the radio to standby! Status: %08X", ret);
            break;
        }

        // set the lora modem parameters
        ral_status = ralf_setup_lora(&ctrl_data->radio, &lora_params);
        if (RAL_STATUS_OK != ral_status)
        {
            ret = -1;
            LOG_ERR("Failed to setup the LoRa modem! Status: %08X", ret);
            break;
        }

        // set the dio irq parameters
        ral_status = ral_set_dio_irq_params(ral_from_ralf(&ctrl_data->radio), RAL_IRQ_TX_DONE);
        if (RAL_STATUS_OK != ral_status)
        {
            ret = -1;
            LOG_ERR("Failed to set the DIO IRQ parameters! Status: %08X", ret);
            break;
        }

        // set the packet payload
        ral_status = ral_set_pkt_payload(ral_from_ralf(&ctrl_data->radio), data , size);
        if (RAL_STATUS_OK != ral_status)
        {
            ret = -1;
            LOG_ERR("Failed to set the packet payload! Status: %08X", ret);
            break;
        }

        switch_to_standby = true;
        // set the radio to transmit
        ral_status = ral_set_tx(ral_from_ralf(&ctrl_data->radio));
        if (RAL_STATUS_OK != ral_status)
        {
            ret = -1;
            LOG_ERR("Failed to set the radio to transmit! Status: %08X", ret);
            break;
        }

        ret = k_sem_take(&ctrl_data->sem_tx_done, K_MSEC(timeout));
        if (ret < 0)
        {
            ret = -ETIMEDOUT;
            LOG_ERR("Transmit timeout!");
            break;
        }

        LOG_INF("Radio is in transmit mode for %u ms", timeout);

    } while(0);

    if (switch_to_standby)
    {
        ral_status = ral_set_standby(ral_from_ralf(&ctrl_data->radio), RAL_STANDBY_CFG_RC);
        if (RAL_STATUS_OK != ral_status)
        {
            ret = -1;
            LOG_ERR("Failed to set the radio to standby! Status: %08X", ret);
        }
    }

    k_mutex_unlock(&ctrl_data->mutex);

    return ret;
}

static int radio_ctrl_impl_receive(const struct device *dev, uint8_t *data, size_t size,
                                   struct msg_stats *stats, uint32_t timeout)
{
    struct radio_ctrl_data *ctrl_data = dev->data;
    int ret = 0;

    k_mutex_lock(&ctrl_data->mutex, K_FOREVER);

    // TODO implement receive logic here using ctrl_data->modem_radio and ctrl_data->rx_params

    k_mutex_unlock(&ctrl_data->mutex);

    return ret;
}

static const struct radio_ctrl_driver_api radio_ctrl_api = {
    .set_type = radio_ctrl_impl_set_type,
    .config = radio_ctrl_impl_config,
    .set_antenna_switch = radio_ctrl_impl_set_antenna_switch,
    .get_lora_rx_params = radio_ctrl_impl_get_lora_rx_params,
    .set_lora_rx_params = radio_ctrl_impl_set_lora_rx_params,
    .get_lora_tx_params = radio_ctrl_impl_get_lora_tx_params,
    .set_lora_tx_params = radio_ctrl_impl_set_lora_tx_params,
    .get_lora_cad_params = radio_ctrl_impl_get_lora_cad_params,
    .set_lora_cad_params = radio_ctrl_impl_set_lora_cad_params,
    .listen = radio_ctrl_impl_listen,
    .cad = radio_ctrl_impl_cad,
    .transmit = radio_ctrl_impl_transmit,
    .receive = radio_ctrl_impl_receive,
};

static void radio_ctrl_irq_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    struct radio_ctrl_data *data = CONTAINER_OF(cb, struct radio_ctrl_data, irq_cb_data);

    k_work_submit(&data->irq_work);
}

static void radio_ctrl_irq_work(struct k_work *work)
{
    struct radio_ctrl_data *data = CONTAINER_OF(work, struct radio_ctrl_data, irq_work);

    // k_mutex_lock(&data->mutex, K_FOREVER);

    LOG_INF("Handling radio IRQ");

    // Read IRQ status from SX1262
    // uint16_t irq_status = sx1262_get_irq_status(...);

    // Handle IRQs: RX done, TX done, CAD done, errors, etc.
    // Update data->stats accordingly

    // k_mutex_unlock(&data->mutex);
}

static int radio_ctrl_init(const struct device *dev)
{
    const struct radio_ctrl_config *cfg = dev->config;
    struct radio_ctrl_data *data = dev->data;
    int ret;

    data->dev = dev;

    k_work_init(&data->irq_work, radio_ctrl_irq_work);

    ret = k_mutex_init(&data->mutex);
    if (ret < 0) {
        LOG_ERR("Failed to initialize mutex");
        return ret;
    }

    ret = k_sem_init(&data->sem_tx_done, 0, 1);
    if (ret < 0) {
        LOG_ERR("Failed to initialize TX done semaphore");
        return ret;
    }
    ret = k_sem_init(&data->sem_irq, 0, 1);
    if (ret < 0) {
        LOG_ERR("Failed to initialize IRQ semaphore");
        return ret;
    }
    ret = k_sem_init(&data->sem_cad_done, 0, 1);
    if (ret < 0) {
        LOG_ERR("Failed to initialize CAD done semaphore");
        return ret;
    }

    if (!device_is_ready(cfg->spi.bus)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }

    if (!device_is_ready(cfg->nreset.port) || !device_is_ready(cfg->busy.port) ||
        !device_is_ready(cfg->irq.port)) {
        LOG_ERR("GPIO not ready");
        return -ENODEV;
    }

    if (cfg->ant_sw.port && !device_is_ready(cfg->ant_sw.port)) {
        LOG_ERR("Antenna switch GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&cfg->nreset, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure NRESET pin");
        return ret;
    }
    ret = gpio_pin_configure_dt(&cfg->busy, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure BUSY pin");
        return ret;
    }
    ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT | GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ pin");
        return ret;
    }
    ret = gpio_pin_configure_dt(&cfg->ant_sw, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure ANT_SW pin");
        return ret;
    }

    gpio_init_callback(&data->irq_cb_data, radio_ctrl_irq_isr, BIT(cfg->irq.pin));
    ret = gpio_add_callback(cfg->irq.port, &data->irq_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add IRQ callback");
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ pin interrupt");
        return ret;
    }

    ret = gpio_pin_set_dt(&cfg->nreset, 1);
    if (ret < 0) {
        LOG_ERR("Failed to set NRESET pin high");
        return ret;
    }
    ret = gpio_pin_set_dt(&cfg->ant_sw, 1);
    if (ret < 0) {
        LOG_ERR("Failed to set ANT_SW pin high");
        return ret;
    }

    ret = radio_ctrl_impl_set_type(dev, RADIO_TYPE_SX126X);
    if (ret < 0) {
        LOG_ERR("Failed to set radio type");
        return ret;
    }

    LOG_INF("SPI: bus=%s op=0x%08x freq=%u slave=%u cs_port=%p cs_pin=%d",
        cfg->spi.bus->name,
        cfg->spi.config.operation,
        cfg->spi.config.frequency,
        cfg->spi.config.slave,
        (void *)cfg->spi.config.cs.gpio.port,
        cfg->spi.config.cs.gpio.pin);

    ral_init(ral_from_ralf(&data->radio));

    LOG_INF("SX1262 radio initialized");

    data->is_initialized = true;

    return 0;
}

#define RADIO_CTRL_INIT(inst)                                                                      \
    static struct radio_ctrl_data radio_ctrl_data_##inst;                                          \
                                                                                                   \
    static const struct radio_ctrl_config radio_ctrl_config_##inst = {                             \
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),                  \
        .nreset = GPIO_DT_SPEC_INST_GET(inst, nreset_gpios),                                       \
        .busy = GPIO_DT_SPEC_INST_GET(inst, busy_gpios),                                           \
        .irq = GPIO_DT_SPEC_INST_GET(inst, dio1_gpios),                                            \
        .ant_sw = GPIO_DT_SPEC_INST_GET_OR(inst, antenna_switch_gpios, {0}),                       \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, radio_ctrl_init, NULL, &radio_ctrl_data_##inst,                    \
                          &radio_ctrl_config_##inst, POST_KERNEL, CONFIG_RADIO_CTRL_INIT_PRIORITY, \
                          &radio_ctrl_api);

DT_INST_FOREACH_STATUS_OKAY(RADIO_CTRL_INIT)
