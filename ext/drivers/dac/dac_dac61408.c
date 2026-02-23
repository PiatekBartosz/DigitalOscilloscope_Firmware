#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(dac_dac61408, CONFIG_DAC_LOG_LEVEL);

#define DT_DRV_COMPAT ti_dac61408

/* ============================================================
 * Register Offsets
 * ============================================================ */
#define DAC61408_REG_NOP              0x00
#define DAC61408_REG_DEVICEID         0x01
#define DAC61408_REG_STATUS           0x02
#define DAC61408_REG_SPICONFIG        0x03
#define DAC61408_REG_GENCONFIG        0x04
#define DAC61408_REG_BRDCONFIG        0x05
#define DAC61408_REG_SYNCCONFIG       0x06
#define DAC61408_REG_TOGGCONFIG0      0x07
#define DAC61408_REG_TOGGCONFIG1      0x08
#define DAC61408_REG_DACPWDWN         0x09
#define DAC61408_REG_DACRANGE0        0x0B
#define DAC61408_REG_DACRANGE1        0x0C
#define DAC61408_REG_TRIGGER          0x0E
#define DAC61408_REG_BRDCAST          0x0F

#define DAC61408_REG_DAC_BASE         0x14   /* DAC0 = 0x14 ... DAC7 = 0x1B */
#define DAC61408_REG_OFFSET0          0x21
#define DAC61408_REG_OFFSET1          0x22

#define DAC61408_REG_DAC(n)          (DAC61408_REG_DAC_BASE + (n))

/* ============================================================
 * STATUS Register (0x02)
 * ============================================================ */
#define DAC61408_STATUS_CRC_ALM_OFFSET     2
#define DAC61408_STATUS_DAC_BUSY_OFFSET    1
#define DAC61408_STATUS_TEMP_ALM_OFFSET    0

#define DAC61408_STATUS_CRC_ALM_MASK     (1U << DAC61408_STATUS_CRC_ALM_OFFSET)
#define DAC61408_STATUS_DAC_BUSY_MASK    (1U << DAC61408_STATUS_DAC_BUSY_OFFSET)
#define DAC61408_STATUS_TEMP_ALM_MASK    (1U << DAC61408_STATUS_TEMP_ALM_OFFSET)

/* ============================================================
 * SPICONFIG Register (0x03)
 * ============================================================ */
#define DAC61408_SPI_TEMPALM_EN_OFFSET     11
#define DAC61408_SPI_DACBUSY_EN_OFFSET     10
#define DAC61408_SPI_CRCALM_EN_OFFSET      9
#define DAC61408_SPI_SOFTTOGGLE_EN_OFFSET  6
#define DAC61408_SPI_DEV_PWDWN_OFFSET      5
#define DAC61408_SPI_CRC_EN_OFFSET         4
#define DAC61408_SPI_STR_EN_OFFSET         3
#define DAC61408_SPI_SDO_EN_OFFSET         2
#define DAC61408_SPI_FSDO_OFFSET           1

#define DAC61408_SPI_TEMPALM_EN_MASK     (1U << DAC61408_SPI_TEMPALM_EN_OFFSET)
#define DAC61408_SPI_DACBUSY_EN_MASK     (1U << DAC61408_SPI_DACBUSY_EN_OFFSET)
#define DAC61408_SPI_CRCALM_EN_MASK      (1U << DAC61408_SPI_CRCALM_EN_OFFSET)
#define DAC61408_SPI_SOFTTOGGLE_EN_MASK  (1U << DAC61408_SPI_SOFTTOGGLE_EN_OFFSET)
#define DAC61408_SPI_DEV_PWDWN_MASK      (1U << DAC61408_SPI_DEV_PWDWN_OFFSET)
#define DAC61408_SPI_CRC_EN_MASK         (1U << DAC61408_SPI_CRC_EN_OFFSET)
#define DAC61408_SPI_STR_EN_MASK         (1U << DAC61408_SPI_STR_EN_OFFSET)
#define DAC61408_SPI_SDO_EN_MASK         (1U << DAC61408_SPI_SDO_EN_OFFSET)
#define DAC61408_SPI_FSDO_MASK           (1U << DAC61408_SPI_FSDO_OFFSET)

/* ============================================================
 * GENCONFIG Register (0x04)
 * ============================================================ */
#define DAC61408_GEN_REF_PWDWN_OFFSET      14
#define DAC61408_GEN_DIFF_67_OFFSET        5
#define DAC61408_GEN_DIFF_45_OFFSET        4
#define DAC61408_GEN_DIFF_23_OFFSET        3
#define DAC61408_GEN_DIFF_01_OFFSET        2

#define DAC61408_GEN_REF_PWDWN_MASK      (1U << DAC61408_GEN_REF_PWDWN_OFFSET)
#define DAC61408_GEN_DIFF_67_MASK        (1U << DAC61408_GEN_DIFF_67_OFFSET)
#define DAC61408_GEN_DIFF_45_MASK        (1U << DAC61408_GEN_DIFF_45_OFFSET)
#define DAC61408_GEN_DIFF_23_MASK        (1U << DAC61408_GEN_DIFF_23_OFFSET)
#define DAC61408_GEN_DIFF_01_MASK        (1U << DAC61408_GEN_DIFF_01_OFFSET)

/* ============================================================
 * BRDCONFIG Register (0x05)
 * ============================================================ */
#define DAC61408_BRDCAST_EN_OFFSET(n)      (4 + (n))
#define DAC61408_BRDCAST_EN_MASK(n)      (1U << DAC61408_BRDCAST_EN_OFFSET(n))

/* ============================================================
 * SYNCCONFIG Register (0x06)
 * ============================================================ */
#define DAC61408_SYNC_EN_OFFSET(n)         (4 + (n))
#define DAC61408_SYNC_EN_MASK(n)         (1U << DAC61408_SYNC_EN_OFFSET(n))

/* ============================================================
 * TOGGCONFIG0 (DAC7–4) (0x07)
 * ============================================================ */
#define DAC61408_TOGG0_FIELD_OFFSET(n)     ((n - 4) * 2)
#define DAC61408_TOGG0_FIELD_MASK(n)     (0x3U << DAC61408_TOGG0_FIELD_OFFSET(n))

/* ============================================================
 * TOGGCONFIG1 (DAC3–0) (0x08)
 * ============================================================ */
#define DAC61408_TOGG1_FIELD_OFFSET(n)     ((n) * 2 + 8)
#define DAC61408_TOGG1_FIELD_MASK(n)     (0x3U << DAC61408_TOGG1_FIELD_OFFSET(n))

/* Toggle Modes */
#define DAC61408_TOGGLE_DISABLED        0x0
#define DAC61408_TOGGLE_MODE0           0x1
#define DAC61408_TOGGLE_MODE1           0x2
#define DAC61408_TOGGLE_MODE2           0x3

/* ============================================================
 * DACPWDWN Register (0x09)
 * ============================================================ */
#define DAC61408_DAC_PWDWN_OFFSET(n)       (4 + (n))
#define DAC61408_DAC_PWDWN_MASK(n)       (1U << DAC61408_DAC_PWDWN_OFFSET(n))

/* ============================================================
 * DACRANGEn (0x0B, 0x0C)
 * ============================================================ */
#define DAC61408_RANGE_FIELD_OFFSET(slot)  ((slot) * 4)
#define DAC61408_RANGE_FIELD_MASK(slot)  (0xFU << DAC61408_RANGE_FIELD_OFFSET(slot))

/* Range codes */
#define DAC61408_RANGE_0_5V             0x0
#define DAC61408_RANGE_0_10V            0x1
#define DAC61408_RANGE_0_20V            0x2
#define DAC61408_RANGE_0_40V            0x4
#define DAC61408_RANGE_PM5V             0x9
#define DAC61408_RANGE_PM10V            0xA
#define DAC61408_RANGE_PM20V            0xC
#define DAC61408_RANGE_PM2_5V           0xE

/* ============================================================
 * TRIGGER Register (0x0E)
 * ============================================================ */
#define DAC61408_TRIG_ALM_RESET_OFFSET     8
#define DAC61408_TRIG_AB_TOG2_OFFSET       7
#define DAC61408_TRIG_AB_TOG1_OFFSET       6
#define DAC61408_TRIG_AB_TOG0_OFFSET       5
#define DAC61408_TRIG_LDAC_OFFSET          4

#define DAC61408_TRIG_ALM_RESET_MASK     (1U << DAC61408_TRIG_ALM_RESET_OFFSET)
#define DAC61408_TRIG_AB_TOG2_MASK       (1U << DAC61408_TRIG_AB_TOG2_OFFSET)
#define DAC61408_TRIG_AB_TOG1_MASK       (1U << DAC61408_TRIG_AB_TOG1_OFFSET)
#define DAC61408_TRIG_AB_TOG0_MASK       (1U << DAC61408_TRIG_AB_TOG0_OFFSET)
#define DAC61408_TRIG_LDAC_MASK          (1U << DAC61408_TRIG_LDAC_OFFSET)

#define DAC61408_TRIG_SOFT_RESET_CODE   0xA
#define DAC61408_TRIG_SOFT_RESET_MASK    0xF

/* ============================================================
 * BRDCAST Register (0x0F)
 * ============================================================ */
#define DAC61408_BRDCAST_DATA_MASK       0xFFFFU

/* ============================================================
 * DACn Register (0x14–0x1B)
 * ============================================================ */
#define DAC61408_DAC_DATA_MASK           0xFFFFU

/* ============================================================
 * OFFSETn Register (0x21–0x22)
 * ============================================================ */
#define DAC61408_OFFSET_AB_OFFSET          8
#define DAC61408_OFFSET_CD_OFFSET          0

#define DAC61408_OFFSET_AB_MASK          (0xFFU << DAC61408_OFFSET_AB_OFFSET)
#define DAC61408_OFFSET_CD_MASK          (0xFFU << DAC61408_OFFSET_CD_OFFSET)
typedef enum
{
	REF_INTERNAL = 0,
	REF_EXTERNAL = 1,
} Voltage_Reference_Source_t;

typedef struct
{
	struct spi_dt_spec bus;
	struct gpio_dt_spec clearGpio;
	struct gpio_dt_spec resetGpio;
	uint8_t reference;
} Dac61408_Config_t;

typedef struct
{
	uint8_t resolution;
} Dac61408_Data_t;

/* ============================================================
 * Low Level SPI Access
 * ============================================================ */

static int dac61408_reg_write(const struct device *dev,
							  uint8_t addr,
							  uint16_t data)
{
	const Dac61408_Config_t *config = dev->config;

	uint8_t tx_buf[3] = {0};
	uint8_t rx_buf[3] = {0};

	/* Bit7 = 0 for WRITE */
	tx_buf[0] = addr & 0x7F;
	sys_put_be16(data, &tx_buf[1]);

	int ret = spi_transceive_dt(&config->bus, &tx_buf, &rx_buf);
	if (ret)
	{
		LOG_ERR("SPI write failed (%d)", ret);
		return ret;
	}

	// second transaction is required
	// ret = spi_transceive_dt(&config->bus, &tx_buf, &rx_buf);
	// if (ret)
	// {
	// 	LOG_ERR("SPI write failed (%d)", ret);
	// 	return ret;
	// }

	LOG_DBG("RD RAW send: 0x%02X 0x%02X 0x%02X", tx_buf[0], tx_buf[1], tx_buf[2]);
	LOG_DBG("RD RAW recv: 0x%02X 0x%02X 0x%02X", rx_buf[0], rx_buf[1], rx_buf[2]);

	// TODO: compare rx and tx and throw when it's bad

	return 0;
}

static int dac61408_reg_read(const struct device *dev,
							 uint8_t addr,
							 uint16_t *data)
{
	const Dac61408_Config_t *config = dev->config;

	uint8_t tx_buf[3];
	uint8_t rx_buf[3] = {0};

	/* Bit7 = 1 for READ */
	tx_buf[0] = addr | 0x80;
	tx_buf[1] = 0x00;
	tx_buf[2] = 0x00;

	struct spi_buf txb = {.buf = tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf rxb = {.buf = rx_buf, .len = sizeof(rx_buf)};

	struct spi_buf_set tx = {.buffers = &txb, .count = 1};
	struct spi_buf_set rx = {.buffers = &rxb, .count = 1};

	int ret = spi_transceive_dt(&config->bus, &tx, &rx);
	if (ret)
	{
		LOG_ERR("SPI read failed (%d)", ret);
		return ret;
	}

	// second transaction is required
	ret = spi_transceive_dt(&config->bus, &tx, &rx);
	if (ret)
	{
		LOG_ERR("SPI read failed (%d)", ret);
		return ret;
	}

	*data = sys_get_be16(&rx_buf[1]);

	LOG_DBG("RD RAW send: 0x%02X 0x%02X 0x%02X", tx_buf[0], tx_buf[1], tx_buf[2]);
	LOG_DBG("RD RAW recv: 0x%02X 0x%02X 0x%02X", rx_buf[0], rx_buf[1], rx_buf[2]);

	return 0;
}

static int dac61408_reg_update_bits(const struct device *dev,
									uint8_t addr,
									uint16_t mask,
									uint16_t value)
{
	uint16_t tmp;
	int ret = dac61408_reg_read(dev, addr, &tmp);
	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= (value & mask);

	return dac61408_reg_write(dev, addr, tmp);
}

/* ============================================================
 * DAC API
 * ============================================================ */

static int dac61408_channel_setup(const struct device *dev,
								  const struct dac_channel_cfg *cfg)
{
	Dac61408_Data_t *data = dev->data;

	if (cfg->channel_id > 7)
		return -EINVAL;

	if (cfg->resolution != data->resolution)
		return -ENOTSUP;

	return 0;
}

static int dac61408_write_value(const struct device *dev,
								uint8_t channel,
								uint32_t value)
{
	Dac61408_Data_t *data = dev->data;

	if (channel > 7)
		return -EINVAL;

	if (value >= (1U << data->resolution))
		return -EINVAL;

	uint16_t reg_val = value << (16 - data->resolution);

	return dac61408_reg_write(dev,
							  DAC61408_REG_DAC(channel),
							  reg_val);
}

/* ============================================================
 * Init
 * ============================================================ */

static int dac61408_init(const struct device *dev)
{
	const Dac61408_Config_t *config = dev->config;
	Dac61408_Data_t *data = dev->data;
	uint16_t device_id;
	uint16_t status;
	int ret;

	LOG_INF("Initializing DAC61408...");

	if (!gpio_is_ready_dt(&config->clearGpio) ||
		!gpio_is_ready_dt(&config->resetGpio) ||
		!spi_is_ready_dt(&config->bus))
	{
		LOG_ERR("Device not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&config->clearGpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&config->resetGpio, GPIO_OUTPUT_INACTIVE);

	/* Soft reset */
	ret = dac61408_reg_write(dev,
							 DAC61408_REG_TRIGGER,
							 DAC61408_TRIG_SOFT_RESET_CODE);
	if (ret)
		return ret;

	k_msleep(5);

	/* Enable SDO so reads work */
	ret = dac61408_reg_update_bits(dev,
								   DAC61408_REG_SPICONFIG,
								   DAC61408_SPI_SDO_EN_MASK,
								   DAC61408_SPI_SDO_EN_MASK);
	if (ret)
		return ret;

	LOG_INF("SDO enabled");

	/* Read device ID */
	ret = dac61408_reg_read(dev,
							DAC61408_REG_DEVICEID,
							&device_id);
	if (ret)
		return ret;

	LOG_INF("Device ID: 0x%04X", device_id);

	/* Read status */
	ret = dac61408_reg_read(dev,
							DAC61408_REG_STATUS,
							&status);
	if (!ret)
		LOG_INF("STATUS: 0x%04X", status);

	data->resolution = 12;

	/* Configure reference */
	uint16_t genconfig = 0;

	if (config->reference == REF_EXTERNAL)
	{
		genconfig |= DAC61408_GEN_REF_PWDWN_MASK;
		LOG_INF("External reference selected");
	}
	else
	{
		LOG_INF("Internal reference selected");
	}

	ret = dac61408_reg_write(dev,
							 DAC61408_REG_GENCONFIG,
							 genconfig);
	if (ret)
		return ret;

	/* Power up all DACs */
	ret = dac61408_reg_write(dev,
							 DAC61408_REG_DACPWDWN,
							 0x0000);
	if (ret)
		return ret;

	LOG_INF("DAC61408 initialization complete");

	return 0;
}

/* ============================================================
 * Driver API
 * ============================================================ */

static DEVICE_API(dac, dac61408_driver_api) = {
	.channel_setup = dac61408_channel_setup,
	.write_value = dac61408_write_value,
};

#define DAC61408_DEFINE(inst)                                               \
	static Dac61408_Data_t dac61408_data_##inst;                            \
	static const Dac61408_Config_t dac61408_config_##inst = {               \
		.reference = DT_INST_PROP_OR(inst, voltage_reference, 0),           \
		.clearGpio = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), clr_gpios),        \
		.resetGpio = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), reset_gpios),      \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                   \
									SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0), \
	};                                                                      \
	DEVICE_DT_INST_DEFINE(inst,                                             \
						  dac61408_init,                                    \
						  NULL,                                             \
						  &dac61408_data_##inst,                            \
						  &dac61408_config_##inst,                          \
						  POST_KERNEL,                                      \
						  CONFIG_DAC_DAC61408_INIT_PRIORITY,                \
						  &dac61408_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC61408_DEFINE)