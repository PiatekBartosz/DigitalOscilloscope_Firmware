#include "afe_manager/afe_manager.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/logging/log.h>

/* Private Macros */

LOG_MODULE_REGISTER(afe_manager);

#define AFE_MANGER_DAC_LIMIT_LOW_DOUBLE (0.0L)
#define AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE (100.0L)
#define AFE_MANGER_DAC_HIGH_CODE 0xFFFu

#define AFE_MANGER_DAC_GAIN_CH1 0u
#define AFE_MANGER_DAC_GAIN_CH2 1u
#define AFE_MANGER_DAC_OFFSET_CH1 2u
#define AFE_MANGER_DAC_OFFSET_CH2 3u

#define AFE_MANGER_DAC_CH1 1u
#define AFE_MANGER_DAC_CH2 2u

/* Private Types */

typedef struct afe_manager_config_s
{
    const struct device *const dacDev;

} afe_manager_config_t;

/* Private Variables */

static afe_manager_config_t afe_manager_config = {
    .dacDev = DEVICE_DT_GET(DT_ALIAS(dac0)),
};

/* Private Function Prototypes */

/* Private Functions */

static int afe_manager_initGpio(void)
{
    return 0;
}

static int afe_manager_initDac(void)
{
    int errorCode = 0;

    do
    {
        if (!device_is_ready(afe_manager_config.dacDev))
        {
            LOG_ERR("DAC device %s is not ready", afe_manager_config.dacDev->name);
            errorCode = -ENODEV;
            break;
        }

    } while(0);

    return errorCode;
}

static uint32_t afe_manager_encodeDac(const double value)
{
    double clamped = value;
    if (clamped < AFE_MANGER_DAC_LIMIT_LOW_DOUBLE) clamped = AFE_MANGER_DAC_LIMIT_LOW_DOUBLE;
    if (clamped > AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE) clamped = AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE;

    double scaled = (clamped / (AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE - AFE_MANGER_DAC_LIMIT_LOW_DOUBLE))
                   * (double)AFE_MANGER_DAC_HIGH_CODE;

    return (uint32_t)(scaled + 0.5L);
}

static double afe_manager_decodeDac(const uint32_t value) __attribute__((unused));
static double afe_manager_decodeDac(const uint32_t value)
{
    double scaled = ((double)value) / (double)AFE_MANGER_DAC_HIGH_CODE;
    return scaled * (AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE - AFE_MANGER_DAC_LIMIT_LOW_DOUBLE);
}

static bool afe_manager_isChannelValid(const uint8_t channel)
{
    bool isValid = true;
    if ((channel != AFE_MANGER_DAC_CH1) || (channel != AFE_MANGER_DAC_CH2))
    {
        LOG_ERR("Channel can be either %u or %u", AFE_MANGER_DAC_CH1, AFE_MANGER_DAC_CH2);
        isValid = false;
    }

    return isValid;
}

/* Public Functions */

int afe_manager_init(void)
{
    int errorCode = 0;

    do
    {
        errorCode = afe_manager_initGpio();
        if (errorCode != 0)
        {
            LOG_ERR("Failed initalizing AFE GPIO error: %d", errorCode);
            break;
        }

        errorCode = afe_manager_initDac();
        if (errorCode != 0)
        {
            LOG_ERR("Failed initalizing AFE GPIO error: %d", errorCode);
            break;
        }

        LOG_INF("Succesfully initalized AFE");

    } while(0);

    return errorCode;  
}

int afe_manager_setGain(const uint8_t channel, const double set_percent)
{
    const uint32_t rawValue = afe_manager_encodeDac(set_percent);
    int errorCode = 0;
    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        const uint8_t absoluteChannel = (channel == AFE_MANGER_DAC_CH1) ? AFE_MANGER_DAC_GAIN_CH1 : AFE_MANGER_DAC_GAIN_CH2;
        errorCode = dac_write_value(afe_manager_config.dacDev, absoluteChannel, rawValue);
        if (errorCode != 0)
        {
            LOG_ERR("Cannot write to dac, error: %d", errorCode);
            break;
        }

    } while(0);

    return errorCode;
}

int afe_manager_setOffset(const uint8_t channel, const double set_percent)
{
    const uint32_t rawValue = afe_manager_encodeDac(set_percent);
    int errorCode = 0;
    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        const uint8_t absoluteChannel = (channel == AFE_MANGER_DAC_CH1) ? AFE_MANGER_DAC_OFFSET_CH1 : AFE_MANGER_DAC_OFFSET_CH2;
        errorCode = dac_write_value(afe_manager_config.dacDev, absoluteChannel, rawValue);
        if (errorCode != 0)
        {
            LOG_ERR("Cannot write to dac, error: %d", errorCode);
            break;
        }

    } while(0);

    return errorCode;
}
