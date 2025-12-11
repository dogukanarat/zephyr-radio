#ifndef ZEPHYR_INCLUDE_DRIVERS_RADIO_CTRL_H_
#define ZEPHYR_INCLUDE_DRIVERS_RADIO_CTRL_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include "zephyr/drivers/ralf/ralf.h"
#include "zephyr/drivers/ral/ral.h"

#ifdef __cplusplus
extern "C" {
#endif

enum radio_type {
    RADIO_TYPE_SX126X,
    RADIO_TYPE_SX127X,
    RADIO_TYPE_SX128X,
    RADIO_TYPE_LR1110,
    RADIO_TYPE_LAST,
};

struct msg_stats {
    int16_t rssi;       // [dBm]
    int16_t snr;        // [dB]
    int16_t signalRssi; // [dBm]
};

typedef int (*radio_ctrl_api_set_type)(const struct device *dev, enum radio_type type);

typedef int (*radio_ctrl_api_config)(const struct device *dev, const ralf_params_lora_t *rx_params,
                                     const ralf_params_lora_t *tx_params,
                                     const ralf_params_lora_cad_t *cad_params);

typedef int (*radio_ctrl_api_set_antenna_switch)(const struct device *dev, bool tx_enable);

typedef int (*radio_ctrl_api_get_lora_rx_params)(const struct device *dev,
                                                 ralf_params_lora_t *params);
typedef int (*radio_ctrl_api_set_lora_rx_params)(const struct device *dev,
                                                 const ralf_params_lora_t *params);
typedef int (*radio_ctrl_api_get_lora_tx_params)(const struct device *dev,
                                                 ralf_params_lora_t *params);
typedef int (*radio_ctrl_api_set_lora_tx_params)(const struct device *dev,
                                                 const ralf_params_lora_t *params);
typedef int (*radio_ctrl_api_set_lora_cad_params)(const struct device *dev,
                                                  const ralf_params_lora_cad_t *params);
typedef int (*radio_ctrl_api_get_lora_cad_params)(const struct device *dev,
                                                  ralf_params_lora_cad_t *params);

typedef int (*radio_ctrl_api_listen)(const struct device *dev, uint32_t timeout);
typedef int (*radio_ctrl_api_cad)(const struct device *dev, uint32_t timeout);
typedef int (*radio_ctrl_api_transmit)(const struct device *dev, const uint8_t *data, size_t size);
typedef int (*radio_ctrl_api_receive)(const struct device *dev, uint8_t *data, size_t size,
                                      struct msg_stats *stats, uint32_t timeout);

__subsystem struct radio_ctrl_driver_api {
    radio_ctrl_api_set_type set_type;
    radio_ctrl_api_config config;
    radio_ctrl_api_set_antenna_switch set_antenna_switch;
    radio_ctrl_api_get_lora_rx_params get_lora_rx_params;
    radio_ctrl_api_set_lora_rx_params set_lora_rx_params;
    radio_ctrl_api_get_lora_tx_params get_lora_tx_params;
    radio_ctrl_api_set_lora_tx_params set_lora_tx_params;
    radio_ctrl_api_get_lora_cad_params get_lora_cad_params;
    radio_ctrl_api_set_lora_cad_params set_lora_cad_params;
    radio_ctrl_api_listen listen;
    radio_ctrl_api_cad cad;
    radio_ctrl_api_transmit transmit;
    radio_ctrl_api_receive receive;
};

static inline int radio_ctrl_set_type(const struct device *dev, enum radio_type type)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_type(dev, type);
}

static inline int radio_ctrl_config(const struct device *dev, const ralf_params_lora_t *rx_params,
                                    const ralf_params_lora_t *tx_params,
                                    const ralf_params_lora_cad_t *cad_params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->config(dev, rx_params, tx_params, cad_params);
}

static inline int radio_ctrl_set_antenna_switch(const struct device *dev, bool tx_enable)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_antenna_switch(dev, tx_enable);
}

static inline int radio_ctrl_get_lora_rx_params(const struct device *dev,
                                                ralf_params_lora_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->get_lora_rx_params(dev, params);
}

static inline int radio_ctrl_set_lora_rx_params(const struct device *dev,
                                                const ralf_params_lora_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_lora_rx_params(dev, params);
}

static inline int radio_ctrl_get_lora_tx_params(const struct device *dev,
                                                ralf_params_lora_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->get_lora_tx_params(dev, params);
}

static inline int radio_ctrl_set_lora_tx_params(const struct device *dev,
                                                const ralf_params_lora_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_lora_tx_params(dev, params);
}

static inline int radio_ctrl_get_lora_cad_params(const struct device *dev,
                                                 ralf_params_lora_cad_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->get_lora_cad_params(dev, params);
}

static inline int radio_ctrl_set_lora_cad_params(const struct device *dev,
                                                 const ralf_params_lora_cad_t *params)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->set_lora_cad_params(dev, params);
}

static inline int radio_ctrl_listen(const struct device *dev, uint32_t timeout)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->listen(dev, timeout);
}

static inline int radio_ctrl_cad(const struct device *dev, uint32_t timeout)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->cad(dev, timeout);
}

static inline int radio_ctrl_transmit(const struct device *dev, const uint8_t *data, size_t size)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->transmit(dev, data, size);
}

static inline int radio_ctrl_receive(const struct device *dev, uint8_t *data, size_t size,
                                     struct msg_stats *statistics, uint32_t timeout)
{
    const struct radio_ctrl_driver_api *api = (const struct radio_ctrl_driver_api *)dev->api;
    return api->receive(dev, data, size, statistics, timeout);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_RADIO_CTRL_H_ */
