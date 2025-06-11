/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "icm20948.h"
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(icm20948, CONFIG_SENSOR_LOG_LEVEL);

/* SPI read/write bit definitions */
#define ICM20948_SPI_READ_BIT   0x80
#define ICM20948_SPI_WRITE_BIT  0x00

static int icm20948_spi_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    const struct icm20948_config *cfg = dev->config;
    uint8_t tx_buf = reg | ICM20948_SPI_READ_BIT;
    const struct spi_buf tx_spi_buf = {
        .buf = &tx_buf,
        .len = 1,
    };
    const struct spi_buf_set tx_spi_buf_set = {
        .buffers = &tx_spi_buf,
        .count = 1,
    };
    struct spi_buf rx_spi_buf[2] = {
        {
            .buf = NULL,  /* Dummy byte for register address */
            .len = 1,
        },
        {
            .buf = data,
            .len = len,
        }
    };
    const struct spi_buf_set rx_spi_buf_set = {
        .buffers = rx_spi_buf,
        .count = 2,
    };
    int ret;

    ret = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (ret < 0) {
        LOG_ERR("SPI read register 0x%02X failed: %d", reg, ret);
        return ret;
    }

    return 0;
}

static int icm20948_spi_write_reg(const struct device *dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    const struct icm20948_config *cfg = dev->config;
    uint8_t tx_buf[len + 1];
    const struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = len + 1,
    };
    const struct spi_buf_set tx_spi_buf_set = {
        .buffers = &tx_spi_buf,
        .count = 1,
    };
    int ret;

    tx_buf[0] = reg | ICM20948_SPI_WRITE_BIT;
    memcpy(&tx_buf[1], data, len);

    ret = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (ret < 0) {
        LOG_ERR("SPI write register 0x%02X failed: %d", reg, ret);
        return ret;
    }

    return 0;
}

/* Single-byte convenience functions - UNUSED, commented out to reduce code size */
/*
static int icm20948_spi_read_reg_single(const struct device *dev, uint8_t reg, uint8_t *data)
{
    return icm20948_spi_read_reg(dev, reg, data, 1);
}

static int icm20948_spi_write_reg_single(const struct device *dev, uint8_t reg, uint8_t data)
{
    return icm20948_spi_write_reg(dev, reg, &data, 1);
}
*/

/* lib/emd transport function for SPI */
static int icm20948_emd_spi_read(void *context, uint8_t reg, uint8_t *data, uint32_t len)
{
    const struct device *dev = (const struct device *)context;
    return icm20948_spi_read_reg(dev, reg, data, len);
}

static int icm20948_emd_spi_write(void *context, uint8_t reg, const uint8_t *data, uint32_t len)
{
    const struct device *dev = (const struct device *)context;
    return icm20948_spi_write_reg(dev, reg, data, len);
}

const struct icm20948_transport_api icm20948_spi_api = {
    .read_reg = icm20948_spi_read_reg,
    .write_reg = icm20948_spi_write_reg,
    /* Single-byte functions commented out - unused */
    /* .read_reg_single = icm20948_spi_read_reg_single, */
    /* .write_reg_single = icm20948_spi_write_reg_single, */
    .emd_read = icm20948_emd_spi_read,
    .emd_write = icm20948_emd_spi_write,
};

int icm20948_spi_init(const struct device *dev)
{
    const struct icm20948_config *cfg = dev->config;
    
    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("SPI bus %s not ready", cfg->spi.bus->name);
        return -ENODEV;
    }

    LOG_DBG("ICM20948 SPI interface initialized on %s", cfg->spi.bus->name);

    return 0;
}

