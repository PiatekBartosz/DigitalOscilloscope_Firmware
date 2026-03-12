#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <string.h>

#include "afe_manager/afe_manager.h"

/* Private helpers */

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

/* Shell commands */

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
    afe_manager_coupling_t coupling = AFE_MANAGER_CH1;

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

static int app_shell_cmdAfeTrigger(const struct shell *shell, size_t argc, char **argv)
{
    app_shell_debugPrintCmd(shell, argc, argv);
    int errorCode = 0;
    afe_manager_coupling_t coupling = AFE_MANAGER_COUPLING_AC;

    do
    {
        errorCode = app_shell_parseCoupling(argv[1], &coupling);
        if (errorCode != 0)
        {
            shell_error(shell, "Coupling must be ac or dc");
            break;
        }

        errorCode = afe_manager_setTriggerCoupling(coupling);
        if (errorCode != 0)
        {
            shell_error(shell, "Failed to set trigger coupling (%d)", errorCode);
            break;
        }

        shell_print(shell, "Trigger coupling updated");

    } while (0);

    return errorCode;
}


/* Command tree */

SHELL_STATIC_SUBCMD_SET_CREATE(
    app_shell_afe_cmds,
    SHELL_CMD_ARG(gain, NULL, "gain <ch> <percent>", app_shell_cmdAfeGain, 3, 0),
    SHELL_CMD_ARG(offset, NULL, "offset <ch> <percent>", app_shell_cmdAfeOffset, 3, 0),
    SHELL_CMD_ARG(atten, NULL, "atten <ch> <1|100>", app_shell_cmdAfeAtten, 3, 0),
    SHELL_CMD_ARG(coupling, NULL, "coupling <ch> <ac|dc>", app_shell_cmdAfeCoupling, 3, 0),
    SHELL_CMD_ARG(trigger, NULL, "trigger <ac|dc>", app_shell_cmdAfeTrigger, 2, 0),
    SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
    app_shell_cmds,
    SHELL_CMD(afe, &app_shell_afe_cmds, "AFE control", NULL),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(app, &app_shell_cmds, "Application commands", NULL);
