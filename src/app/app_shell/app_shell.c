#include <stdlib.h>
#include <string.h>
#include <zephyr/shell/shell.h>

#include "afe_manager/afe_manager.h"
#include "app.h"
#include "parallel_link/parallel_link.h"

static void app_shell_debugPrintCmd(const struct shell *shell, size_t argc, char **argv)
{
    shell_fprintf(shell, SHELL_NORMAL, "CMD:");

    for (size_t i = 0; i < argc; i++)
    {
        shell_fprintf(shell, SHELL_NORMAL, " %s", argv[i]);
    }

    shell_fprintf(shell, SHELL_NORMAL, "\n");
}

static int app_shell_parseChannel(const char *arg, afe_manager_channel_t *channel)
{
    int errorCode = 0;

    do
    {
        const int value = atoi(arg);

        if (value == 1)
        {
            *channel = AFE_MANAGER_CH1;
        }
        else if (value == 2)
        {
            *channel = AFE_MANAGER_CH2;
        }
        else
        {
            errorCode = -EINVAL;
            break;
        }

    } while (0);

    return errorCode;
}

static int app_shell_parseCoupling(const char *arg, afe_manager_coupling_t *coupling)
{
    int errorCode = 0;

    do
    {
        if (strcmp(arg, "ac") == 0)
        {
            *coupling = AFE_MANAGER_COUPLING_AC;
        }
        else if (strcmp(arg, "dc") == 0)
        {
            *coupling = AFE_MANAGER_COUPLING_DC;
        }
        else
        {
            errorCode = -EINVAL;
            break;
        }

    } while (0);

    return errorCode;
}

static int app_shell_parseAttenuation(const char *arg, afe_manager_attenuation_t *attenuation)
{
    int errorCode = 0;

    do
    {
        const int value = atoi(arg);

        if (value == 1)
        {
            *attenuation = AFE_MANAGER_ATTEN_1_TO_1;
        }
        else if (value == 100)
        {
            *attenuation = AFE_MANAGER_ATTEN_1_TO_100;
        }
        else
        {
            errorCode = -EINVAL;
            break;
        }

    } while (0);

    return errorCode;
}

static int app_shell_cmdAfeGain(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);
    int errorCode = 0;
    afe_manager_channel_t channel = AFE_MANAGER_CH1;
    float percent = 0.0f;

    do
    {
        errorCode = app_shell_parseChannel(argv[1], &channel);
        if (errorCode != 0)
        {
            shell_error(shell, "Invalid channel (1 or 2)");
            break;
        }

        percent = strtof(argv[2], NULL);

        errorCode = afe_manager_setGain(channel, percent);
        if (errorCode != 0)
        {
            shell_error(shell, "Failed to set gain (%d)", errorCode);
            break;
        }

        shell_print(shell, "Gain set CH%d -> %.2f%%", (int)channel + 1, (double)percent);

    } while (0);

    return errorCode;
}

static int app_shell_cmdAfeOffset(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);
    int errorCode = 0;
    afe_manager_channel_t channel = AFE_MANAGER_CH1;
    float percent = 0.0f;

    do
    {
        errorCode = app_shell_parseChannel(argv[1], &channel);
        if (errorCode != 0)
        {
            shell_error(shell, "Invalid channel");
            break;
        }

        percent = strtof(argv[2], NULL);

        errorCode = afe_manager_setOffset(channel, percent);
        if (errorCode != 0)
        {
            shell_error(shell, "Failed to set offset (%d)", errorCode);
            break;
        }

        shell_print(shell, "Offset set CH%d -> %.2f%%", (int)channel + 1, (double)percent);

    } while (0);

    return errorCode;
}

static int app_shell_cmdAfeAtten(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);
    int errorCode = 0;
    afe_manager_channel_t channel = AFE_MANAGER_CH1;
    afe_manager_attenuation_t attenuation = AFE_MANAGER_ATTEN_1_TO_1;

    do
    {
        errorCode = app_shell_parseChannel(argv[1], &channel);
        if (errorCode != 0)
        {
            shell_error(shell, "Invalid channel");
            break;
        }

        errorCode = app_shell_parseAttenuation(argv[2], &attenuation);
        if (errorCode != 0)
        {
            shell_error(shell, "Attenuation must be 1 or 100");
            break;
        }

        errorCode = afe_manager_setAttenuation(channel, attenuation);
        if (errorCode != 0)
        {
            shell_error(shell, "Failed to set attenuation (%d)", errorCode);
            break;
        }

        shell_print(shell, "Attenuation set CH%d", channel);

    } while (0);

    return errorCode;
}

static int app_shell_cmdAfeCoupling(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);
    int errorCode = 0;
    afe_manager_channel_t channel = AFE_MANAGER_CH1;
    afe_manager_coupling_t coupling = AFE_MANAGER_COUPLING_AC;

    do
    {
        errorCode = app_shell_parseChannel(argv[1], &channel);
        if (errorCode != 0)
        {
            shell_error(shell, "Invalid channel");
            break;
        }

        errorCode = app_shell_parseCoupling(argv[2], &coupling);
        if (errorCode != 0)
        {
            shell_error(shell, "Coupling must be ac or dc");
            break;
        }

        errorCode = afe_manager_setCoupling(channel, coupling);
        if (errorCode != 0)
        {
            shell_error(shell, "Failed to set coupling (%d)", errorCode);
            break;
        }

        shell_print(shell, "Coupling set CH%d", channel);

    } while (0);

    return errorCode;
}

static int app_shell_cmdAfeTriggerLevel(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);

    float percent = strtof(argv[1], NULL);

    int errorCode = afe_manager_setTriggerLevel(percent);
    if (errorCode != 0)
    {
        shell_error(shell, "Failed to set trigger level (%d)", errorCode);
        return errorCode;
    }

    shell_print(shell, "Trigger level set to %.2f%%", (double)percent);
    return 0;
}

static int app_shell_cmdAfeRange(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);
    afe_manager_channel_t channel;
    int errorCode = app_shell_parseChannel(argv[1], &channel);
    int range = atoi(argv[2]);
    if (errorCode != 0 || (range != 1 && range != 2))
    {
        shell_error(shell, "Usage: range <1|2> <1|2> (differential Vpp)");
        return -EINVAL;
    }

    errorCode = afe_manager_setAdcRange(channel, (afe_manager_adc_range_t)range);
    if (errorCode != 0)
    {
        shell_error(shell, "Failed to set ADC range (%d)", errorCode);
        return errorCode;
    }

    shell_print(shell, "CH%d ADC range set to %d Vpp differential", (int)channel, range);
    return 0;
}

static int app_shell_cmdAfeTriggerCh(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);
    int errorCode = 0;
    afe_manager_channel_t channel = AFE_MANAGER_CH1;

    do
    {
        errorCode = app_shell_parseChannel(argv[1], &channel);
        if (errorCode != 0)
        {
            shell_error(shell, "Channel must be 1 or 2");
            break;
        }

        errorCode = afe_manager_setTriggerSource(channel);
        if (errorCode != 0)
        {
            shell_error(shell, "Failed to set trigger source (%d)", errorCode);
            break;
        }

        shell_print(shell, "Trigger source set to CH%d", (int)channel);

    } while (0);

    return errorCode;
}

static int app_shell_cmdAfeInterleaved(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);

    int errorCode = 0;
    int value = 0;

    do
    {
        value = atoi(argv[1]);

        if (value != 0 && value != 1)
        {
            shell_error(shell, "Value must be 0 or 1");
            errorCode = -EINVAL;
            break;
        }

        errorCode = afe_manager_setInterleaved(value ? true : false);
        if (errorCode != 0)
        {
            shell_error(shell, "Failed to set interleaved (%d)", errorCode);
            break;
        }

        shell_print(shell, "Interleaved %s", value ? "enabled" : "disabled");

    } while (0);

    return errorCode;
}

static int app_shell_cmdAfeMockAdc(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);

    int value = atoi(argv[1]);
    if (value != 0 && value != 1)
    {
        shell_error(shell, "Value must be 0 or 1");
        return -EINVAL;
    }

    int errorCode = app_set_mock_adc(value != 0);
    if (errorCode != 0)
    {
        shell_error(shell, "Failed to set mock ADC (%d)", errorCode);
        return errorCode;
    }

    shell_print(shell, "Mock ADC %s", value ? "enabled" : "disabled");
    return 0;
}

static int app_shell_cmdAfeTriggerMode(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);

    bool enable;
    if (strcmp(argv[1], "normal") == 0)
    {
        enable = true;
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        enable = false;
    }
    else
    {
        shell_error(shell, "Mode must be 'off' or 'normal'");
        return -EINVAL;
    }

    int errorCode = app_set_trigger_mode(enable);
    if (errorCode != 0)
    {
        shell_error(shell, "Failed to set trigger mode (%d)", errorCode);
        return errorCode;
    }

    shell_print(shell, "Trigger mode set to %s", argv[1]);
    return 0;
}

static int app_shell_cmdAfeDecim(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);

    int value = atoi(argv[1]);
    if (value < 1 || value > 1023)
    {
        shell_error(shell, "Decimation factor must be in [1, 1023]");
        return -EINVAL;
    }

    int errorCode = app_set_decim_factor((uint16_t)value);
    if (errorCode != 0)
    {
        shell_error(shell, "Failed to set decimation factor (%d)", errorCode);
        return errorCode;
    }

    shell_print(shell, "Decimation factor set to %d", value);
    return 0;
}

static int app_shell_cmdAfeSampleSize(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);

    int value = atoi(argv[1]);
    if (value <= 0 || value > 8192)
    {
        shell_error(shell, "Count must be a power of two in [1, 8192]");
        return -EINVAL;
    }

    int errorCode = app_set_sample_size((uint16_t)value);
    if (errorCode != 0)
    {
        shell_error(shell, "Failed to set sample size (%d)", errorCode);
        return errorCode;
    }

    shell_print(shell, "Sample size set to %d", value);
    return 0;
}

static int app_shell_cmdAfePretrigger(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);

    int value = atoi(argv[1]);
    if (value < 0 || value > PLINK_PRETRIGGER_MAX || (value != 0 && (value & (value - 1)) != 0))
    {
        shell_error(shell, "Count must be 0 (disabled) or a power of two in [1, %u]", PLINK_PRETRIGGER_MAX);
        return -EINVAL;
    }

    int errorCode = app_set_pretrigger((uint16_t)value);
    if (errorCode != 0)
    {
        shell_error(shell, "Failed to set pretrigger (%d)", errorCode);
        return errorCode;
    }

    shell_print(shell, "Pretrigger set to %d samples", value);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    app_shell_afe_cmds, SHELL_CMD_ARG(gain, NULL, "gain <ch> <percent>", app_shell_cmdAfeGain, 3, 0),
    SHELL_CMD_ARG(offset, NULL, "offset <ch> <percent>", app_shell_cmdAfeOffset, 3, 0),
    SHELL_CMD_ARG(atten, NULL, "atten <ch> <1|100>", app_shell_cmdAfeAtten, 3, 0),
    SHELL_CMD_ARG(coupling, NULL, "coupling <ch> <ac|dc>", app_shell_cmdAfeCoupling, 3, 0),
    SHELL_CMD_ARG(range, NULL, "range <ch> <1|2> (differential Vpp)", app_shell_cmdAfeRange, 3, 0),
    SHELL_CMD_ARG(trigger_ch, NULL, "trigger_ch <1|2>", app_shell_cmdAfeTriggerCh, 2, 0),
    SHELL_CMD_ARG(trigger_level, NULL, "trigger_level <percent 0..100>", app_shell_cmdAfeTriggerLevel, 2, 0),
    SHELL_CMD_ARG(trigger_mode, NULL, "trigger_mode <off|normal>", app_shell_cmdAfeTriggerMode, 2, 0),
    SHELL_CMD_ARG(interleaved, NULL, "interleaved <0|1>", app_shell_cmdAfeInterleaved, 2, 0),
    SHELL_CMD_ARG(mock_adc, NULL, "mock_adc <0|1>", app_shell_cmdAfeMockAdc, 2, 0),
    SHELL_CMD_ARG(sample_size, NULL, "sample_size <count> (power of 2, 1..8192)", app_shell_cmdAfeSampleSize, 2, 0),
    SHELL_CMD_ARG(pretrigger, NULL, "pretrigger <count> (0=disabled, power of 2, <=4096)", app_shell_cmdAfePretrigger,
                  2, 0),
    SHELL_CMD_ARG(decim, NULL, "decim <factor> (1..1023; 1=no decimation)", app_shell_cmdAfeDecim, 2, 0),
    SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(app_shell_cmds, SHELL_CMD(afe, &app_shell_afe_cmds, "AFE control", NULL),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(app, &app_shell_cmds, "Application commands", NULL);
