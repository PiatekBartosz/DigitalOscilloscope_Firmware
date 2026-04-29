#include "afe_manager/afe_manager.h"

#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

/* Private Macros */

LOG_MODULE_REGISTER(afe_manager);

#define AFE_MANGER_DAC_LIMIT_LOW_DOUBLE  (0.0L)
#define AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE (100.0L)
#define AFE_MANGER_DAC_HIGH_CODE         0xFFFu

#define AFE_MANGER_DAC_GAIN_CH1          0u
#define AFE_MANGER_DAC_GAIN_CH2          1u
#define AFE_MANGER_DAC_OFFSET_CH1        2u
#define AFE_MANGER_DAC_OFFSET_CH2        3u

/* Private Types */

typedef struct afe_manager_config_s
{
    const struct device *const dacDev;
    const struct gpio_dt_spec attenuationCh1Gpio;
    const struct gpio_dt_spec attenuationCh2Gpio;
    const struct gpio_dt_spec couplingCh1Gpio;
    const struct gpio_dt_spec couplingCh2Gpio;
    const struct gpio_dt_spec couplingTriggerGpio;
    const struct gpio_dt_spec interleavedGpio;

} afe_manager_config_t;

/* Private Variables */

static afe_manager_state_t afe_manager_state = {
    .ch[0] = {.gain_percent = 0.0f,
              .offset_percent = 0.0f,
              .attenuation = AFE_MANAGER_ATTEN_1_TO_1,
              .coupling = AFE_MANAGER_COUPLING_DC},
    .ch[1] = {.gain_percent = 0.0f,
              .offset_percent = 0.0f,
              .attenuation = AFE_MANAGER_ATTEN_1_TO_1,
              .coupling = AFE_MANAGER_COUPLING_DC},
    .trigger_coupling = AFE_MANAGER_COUPLING_DC,
    .interleaved = false,
};

static afe_manager_config_t afe_manager_config = {
    .dacDev = DEVICE_DT_GET(DT_ALIAS(dac0)),

    .attenuationCh1Gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(attenuation_ch1), gpios),
    .attenuationCh2Gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(attenuation_ch2), gpios),

    .couplingCh1Gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(coupling_ch1), gpios),
    .couplingCh2Gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(coupling_ch2), gpios),

    .couplingTriggerGpio = GPIO_DT_SPEC_GET(DT_NODELABEL(coupling_trigger), gpios),

    .interleavedGpio = GPIO_DT_SPEC_GET(DT_NODELABEL(interleaved), gpios),
};

/* Private Function Prototypes */

/* Private Functions */

static int afe_manager_initGpio(void)
{
    int errorCode = 0;

    const struct gpio_dt_spec *gpios[] = {
        &afe_manager_config.attenuationCh1Gpio,  &afe_manager_config.attenuationCh2Gpio,
        &afe_manager_config.couplingCh1Gpio,     &afe_manager_config.couplingCh2Gpio,
        &afe_manager_config.couplingTriggerGpio, &afe_manager_config.interleavedGpio};
    do
    {
        for (size_t i = 0; i < ARRAY_SIZE(gpios); i++)
        {
            if (!device_is_ready(gpios[i]->port))
            {
                errorCode = -ENODEV;
                LOG_ERR("Gpio device %d is not ready", errorCode);
                break;
            }
        }
        if (errorCode != 0)
        {
            break;
        }

        for (size_t i = 0; i < ARRAY_SIZE(gpios); i++)
        {
            errorCode = gpio_pin_configure_dt(gpios[i], GPIO_OUTPUT_INACTIVE);
            if (errorCode != 0)
            {
                LOG_ERR("Device %d cannot be configured", errorCode);
                break;
            }
        }

    } while (0);

    return errorCode;
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

    } while (0);

    return errorCode;
}

static uint32_t afe_manager_encodeDac(const double value)
{
    double clamped = value;
    if (clamped < AFE_MANGER_DAC_LIMIT_LOW_DOUBLE)
        clamped = AFE_MANGER_DAC_LIMIT_LOW_DOUBLE;
    if (clamped > AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE)
        clamped = AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE;

    double scaled = (clamped / (AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE - AFE_MANGER_DAC_LIMIT_LOW_DOUBLE)) *
                    (double)AFE_MANGER_DAC_HIGH_CODE;

    return (uint32_t)(scaled + 0.5L);
}

static double afe_manager_decodeDac(const uint32_t value) __attribute__((unused));
static double afe_manager_decodeDac(const uint32_t value)
{
    double scaled = ((double)value) / (double)AFE_MANGER_DAC_HIGH_CODE;
    return scaled * (AFE_MANGER_DAC_LIMIT_HIGH_DOUBLE - AFE_MANGER_DAC_LIMIT_LOW_DOUBLE);
}

static bool afe_manager_isChannelValid(const afe_manager_channel_t channel)
{
    bool isValid = true;
    if ((channel != AFE_MANAGER_CH1) && (channel != AFE_MANAGER_CH2))
    {
        LOG_ERR("Channel can be either %u or %u", AFE_MANAGER_CH1, AFE_MANAGER_CH2);
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

    } while (0);

    return errorCode;
}

int afe_manager_setGain(const afe_manager_channel_t channel, const float percent)
{
    int errorCode = 0;
    uint32_t rawValue = 0;
    uint8_t absoluteChannel = 0;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        rawValue = afe_manager_encodeDac(percent);

        absoluteChannel = (channel == AFE_MANAGER_CH1) ? AFE_MANGER_DAC_GAIN_CH1 : AFE_MANGER_DAC_GAIN_CH2;

        errorCode = dac_write_value(afe_manager_config.dacDev, absoluteChannel, rawValue);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set gain DAC value, error: %d", errorCode);
            break;
        }

        afe_manager_state.ch[channel - 1].gain_percent = percent;

    } while (0);

    return errorCode;
}

int afe_manager_setOffset(const afe_manager_channel_t channel, const float percent)
{
    int errorCode = 0;
    uint32_t rawValue = 0;
    uint8_t absoluteChannel = 0;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        rawValue = afe_manager_encodeDac(percent);

        absoluteChannel = (channel == AFE_MANAGER_CH1) ? AFE_MANGER_DAC_OFFSET_CH1 : AFE_MANGER_DAC_OFFSET_CH2;

        errorCode = dac_write_value(afe_manager_config.dacDev, absoluteChannel, rawValue);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set offset DAC value, error: %d", errorCode);
            break;
        }

        afe_manager_state.ch[channel - 1].offset_percent = percent;

    } while (0);

    return errorCode;
}

int afe_manager_setAttenuation(const afe_manager_channel_t channel, const afe_manager_attenuation_t attenuation)
{
    int errorCode = 0;
    const struct gpio_dt_spec *gpio = NULL;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        gpio = (channel == AFE_MANAGER_CH1) ? &afe_manager_config.attenuationCh1Gpio
                                            : &afe_manager_config.attenuationCh2Gpio;

        errorCode = gpio_pin_set_dt(gpio, (attenuation == AFE_MANAGER_ATTEN_1_TO_100) ? 1 : 0);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set attenuation, error: %d", errorCode);
            break;
        }

        afe_manager_state.ch[channel - 1].attenuation = attenuation;

    } while (0);

    return errorCode;
}

int afe_manager_setCoupling(const afe_manager_channel_t channel, const afe_manager_coupling_t coupling)
{
    int errorCode = 0;
    const struct gpio_dt_spec *gpio = NULL;

    do
    {
        if (!afe_manager_isChannelValid(channel))
        {
            errorCode = -EINVAL;
            break;
        }

        gpio = (channel == AFE_MANAGER_CH1) ? &afe_manager_config.couplingCh1Gpio : &afe_manager_config.couplingCh2Gpio;

        errorCode = gpio_pin_set_dt(gpio, (coupling == AFE_MANAGER_COUPLING_DC) ? 1 : 0);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set coupling, error: %d", errorCode);
            break;
        }

        afe_manager_state.ch[channel - 1].coupling = coupling;

    } while (0);

    return errorCode;
}

int afe_manager_setTriggerCoupling(const afe_manager_coupling_t coupling)
{
    int errorCode = 0;

    do
    {
        errorCode =
            gpio_pin_set_dt(&afe_manager_config.couplingTriggerGpio, (coupling == AFE_MANAGER_COUPLING_DC) ? 1 : 0);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set trigger coupling, error: %d", errorCode);
            break;
        }

        afe_manager_state.trigger_coupling = coupling;

    } while (0);

    return errorCode;
}

int afe_manager_setInterleaved(const bool isInterleaved)
{
    int errorCode = 0;

    do
    {
        errorCode = gpio_pin_set_dt(&afe_manager_config.interleavedGpio, isInterleaved ? 1 : 0);
        if (errorCode != 0)
        {
            LOG_ERR("Failed to set interleaved gpio, error: %d", errorCode);
            break;
        }

        afe_manager_state.interleaved = isInterleaved;

    } while (0);

    return errorCode;
}

afe_manager_state_t afe_manager_getState(void)
{
    return afe_manager_state;
}