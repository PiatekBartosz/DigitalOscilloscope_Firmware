#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(dac_dacx1408, CONFIG_DAC_LOG_LEVEL);

#define DACX1408_REG_DEVICE_ID 0x01U
#define DACX1408_REG_SYNC 0x02U
#define DACX1408_REG_CONFIG 0x03U
#define DACX1408_REG_GAIN 0x04U
#define DACX1408_REG_TRIGGER 0x05U
#define DACX1408_REG_STATUS 0x07U
#define DACX1408_REG_DAC 0x08U

#define DACX1408_MASK_DEVICE_ID_RES GENMASK(14, 12)
#define DACX1408_MASK_CONFIG_REF_PWDWN BIT(8)
#define DACX1408_MASK_CONFIG_DAC_PWDWN BIT(0)
#define DACX1408_MASK_GAIN_BUFF_GAIN BIT(0)
#define DACX1408_MASK_GAIN_REFDIV_EN BIT(8)
#define DACX1408_MASK_TRIGGER_SOFT_RESET (BIT(1) | BIT(3))
#define DACX1408_MASK_STATUS_REF_ALM BIT(0)

/* Specifies the source of the reference voltage. */
enum voltage_reference_source
{
	REF_INTERNAL, /* Internal 2.5V voltage reference. */
	REF_EXTERNAL, /* External pin voltage reference. */
};

/* Specifies the reference voltage multiplier. */
enum output_gain
{
	VM_MUL2, /* Multiplies by 2. */
	VM_MUL1, /* Multiplies by 1. */
	VM_DIV2, /* Multiplies by 0.5 */
};

struct dacx1408_config
{
	struct i2c_dt_spec i2c_spec;
	enum voltage_reference_source voltage_reference;
	enum output_gain output_gain;
};

struct dacx1408_data
{
	/* Number of bits in the DAC register: Either 12, 14 or 16. */
	uint8_t resolution;
};

static int dacx1408_reg_read(const struct device *dev, const uint8_t addr, uint16_t *data)
{
	const struct dacx1408_config *config = dev->config;
	uint8_t raw_data[2];
	int status;

	status = i2c_write_read_dt(&config->i2c_spec, &addr, sizeof(addr), raw_data,
							   sizeof(raw_data));
	if (status != 0)
	{
		return status;
	}

	/* DAC is big endian. */
	*data = sys_get_be16(raw_data);
	return 0;
}

static int dacx1408_reg_write(const struct device *dev, uint8_t addr, uint16_t data)
{
	const struct dacx1408_config *config = dev->config;
	uint8_t write_cmd[3] = {addr};

	/* DAC is big endian. */
	sys_put_be16(data, write_cmd + 1);

	return i2c_write_dt(&config->i2c_spec, write_cmd, sizeof(write_cmd));
}

static int dacx1408_channel_setup(const struct device *dev,
								  const struct dac_channel_cfg *channel_cfg)
{
	struct dacx1408_data *data = dev->data;

	/* DACx1408 series only has a single output channel. */
	if (channel_cfg->channel_id != 0)
	{
		LOG_ERR("Unsupported channel %d", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->resolution != data->resolution)
	{
		LOG_ERR("Unsupported resolution %d. Actual: %d", channel_cfg->resolution,
				data->resolution);
		return -ENOTSUP;
	}

	if (channel_cfg->internal)
	{
		LOG_ERR("Internal channels not supported");
		return -ENOTSUP;
	}

	return 0;
}

static int dacx1408_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	struct dacx1408_data *data = dev->data;

	if (channel != 0)
	{
		LOG_ERR("dacx1408: Unsupported channel %d", channel);
		return -ENOTSUP;
	}

	if (value >= (1 << data->resolution))
	{
		LOG_ERR("dacx1408: Value %d out of range", value);
		return -EINVAL;
	}

	value <<= (16 - data->resolution);

	return dacx1408_reg_write(dev, DACX1408_REG_DAC, value);
}

static int dacx1408_init(const struct device *dev)
{
	// const struct dacx1408_config *config = dev->config;
	// struct dacx1408_data *data = dev->data;
	// uint16_t device_id;
	// int status;

	// if (!i2c_is_ready_dt(&config->i2c_spec))
	// {
	// 	LOG_ERR("I2C bus %s not ready", config->i2c_spec.bus->name);
	// 	return -ENODEV;
	// }

	// status = dacx1408_reg_read(dev, DACX1408_REG_DEVICE_ID, &device_id);
	// if (status != 0)
	// {
	// 	LOG_ERR("read DEVICE_ID register failed");
	// 	return status;
	// }

	// /* See DEVICE_ID register RES field in the data sheet. */
	// data->resolution = 16 - 2 * FIELD_GET(DACX1408_MASK_DEVICE_ID_RES, device_id);

	// status = dacx1408_reg_write(dev, DACX1408_REG_CONFIG,
	// 							FIELD_PREP(DACX1408_MASK_CONFIG_REF_PWDWN,
	// 									   config->voltage_reference == REF_EXTERNAL));
	// if (status != 0) {
	// 	LOG_ERR("write CONFIG register failed");
	// 	return status;
	// }

	// status = dacx1408_reg_write(
	// 	dev, DACX1408_REG_GAIN,
	// 	FIELD_PREP(DACX1408_MASK_GAIN_REFDIV_EN, config->output_gain == VM_DIV2) |
	// 		FIELD_PREP(DACX1408_MASK_GAIN_BUFF_GAIN, config->output_gain == VM_MUL2));
	// if (status != 0) {
	// 	LOG_ERR("GAIN Register update failed");
	// 	return status;
	// }

	LOG_ERR("Got here");

	return 0;
}

static DEVICE_API(dac, dacx1408_driver_api) = {
	.channel_setup = dacx1408_channel_setup,
	.write_value = dacx1408_write_value,
};

#define DT_DRV_COMPAT ti_dacx1408

#define DACX1408_DEFINE(n)                                                                   \
	static struct dacx1408_data dacx1408_data_##n = {};                                      \
	static const struct dacx1408_config dacx1408_config_##n = {                              \
		.i2c_spec = I2C_DT_SPEC_INST_GET(n),                                                 \
		.voltage_reference =                                                                 \
			_CONCAT(REF_, DT_STRING_UPPER_TOKEN(DT_DRV_INST(n), voltage_reference)),         \
		.output_gain = _CONCAT(VM_, DT_STRING_UPPER_TOKEN(DT_DRV_INST(n), output_gain)),     \
	};                                                                                       \
	DEVICE_DT_INST_DEFINE(n, &dacx1408_init, NULL, &dacx1408_data_##n, &dacx1408_config_##n, \
						  POST_KERNEL, CONFIG_DAC_DACX1408_INIT_PRIORITY,                    \
						  &dacx1408_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DACX1408_DEFINE)
