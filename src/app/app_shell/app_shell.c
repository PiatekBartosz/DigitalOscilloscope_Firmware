#include <app_shell/app_shell.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/shell/shell.h>

#include <zephyr/logging/log.h>

#include <stdlib.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(app_shell, LOG_LEVEL_INF);

/* ========================= */
/*       Configuration       */
/* ========================= */

typedef struct app_shell_config_s
{
#if DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_shell_uart), okay)
    const struct device *shell_dev;
#endif
} app_shell_config_t;

static app_shell_config_t app_shell_config =
{
#if DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_shell_uart), okay)
    .shell_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart)),
#endif
};

/* ========================= */
/*  Hardware Control Stubs   */
/* ========================= */

static void hw_set_gain(uint8_t ch, int gain)
{
    LOG_INF("HW: Set CH%d gain to %d", ch, gain);
}

static void hw_set_offset(uint8_t ch, int offset)
{
    LOG_INF("HW: Set CH%d offset to %d", ch, offset);
}

static void hw_set_coupling(uint8_t ch, bool ac)
{
    LOG_INF("HW: Set CH%d coupling to %s", ch, ac ? "AC" : "DC");
}

static void hw_set_cso(bool enable)
{
    LOG_INF("HW: CSO %s", enable ? "ON" : "OFF");
}

/* ========================= */
/*        Utilities          */
/* ========================= */

static bool parse_channel(const char *arg, uint8_t *ch)
{
    char *endptr;
    long val = strtol(arg, &endptr, 10);

    if (*endptr != '\0') {
        return false;
    }

    if (val < 1 || val > 2) {
        return false;
    }

    *ch = (uint8_t)val;
    return true;
}

/* ========================= */
/*     Shell Commands        */
/* ========================= */

/* scope gain <ch> <value> */
static int cmd_scope_gain(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_help(sh);
        return -EINVAL;
    }

    uint8_t ch;

    if (!parse_channel(argv[1], &ch)) {
        shell_error(sh, "Invalid channel. Use 1 or 2.");
        return -EINVAL;
    }

    int gain = strtol(argv[2], NULL, 10);

    hw_set_gain(ch, gain);

    shell_print(sh, "CH%d gain set to %d", ch, gain);

    return 0;
}

/* scope offset <ch> <value> */
static int cmd_scope_offset(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_help(sh);
        return -EINVAL;
    }

    uint8_t ch;

    if (!parse_channel(argv[1], &ch)) {
        shell_error(sh, "Invalid channel. Use 1 or 2.");
        return -EINVAL;
    }

    int offset = strtol(argv[2], NULL, 10);

    hw_set_offset(ch, offset);

    shell_print(sh, "CH%d offset set to %d", ch, offset);

    return 0;
}

/* scope coupling <ch> <ac|dc> */
static int cmd_scope_coupling(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_help(sh);
        return -EINVAL;
    }

    uint8_t ch;

    if (!parse_channel(argv[1], &ch)) {
        shell_error(sh, "Invalid channel. Use 1 or 2.");
        return -EINVAL;
    }

    if (strcmp(argv[2], "ac") == 0) {
        hw_set_coupling(ch, true);
        shell_print(sh, "CH%d coupling set to AC", ch);
    }
    else if (strcmp(argv[2], "dc") == 0) {
        hw_set_coupling(ch, false);
        shell_print(sh, "CH%d coupling set to DC", ch);
    }
    else {
        shell_error(sh, "Invalid mode. Use ac or dc.");
        return -EINVAL;
    }

    return 0;
}

/* scope cso <on|off> */
static int cmd_scope_cso(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_help(sh);
        return -EINVAL;
    }

    if (strcmp(argv[1], "on") == 0) {
        hw_set_cso(true);
        shell_print(sh, "CSO enabled");
    }
    else if (strcmp(argv[1], "off") == 0) {
        hw_set_cso(false);
        shell_print(sh, "CSO disabled");
    }
    else {
        shell_error(sh, "Invalid option. Use on/off.");
        return -EINVAL;
    }

    return 0;
}

/* ========================= */
/*   Shell Command Tree      */
/* ========================= */

SHELL_STATIC_SUBCMD_SET_CREATE(sub_scope,
    SHELL_CMD_ARG(gain, NULL,
        "Set gain: scope gain <ch> <value>",
        cmd_scope_gain, 3, 0),

    SHELL_CMD_ARG(offset, NULL,
        "Set offset: scope offset <ch> <value>",
        cmd_scope_offset, 3, 0),

    SHELL_CMD_ARG(coupling, NULL,
        "Set coupling: scope coupling <ch> <ac|dc>",
        cmd_scope_coupling, 3, 0),

    SHELL_CMD_ARG(cso, NULL,
        "Control CSO: scope cso <on|off>",
        cmd_scope_cso, 2, 0),

    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(scope,
                   &sub_scope,
                   "Digital oscilloscope control",
                   NULL);

/* ========================= */
/*        Init               */
/* ========================= */

int app_shell_init(void)
{
    int errorCode = 0;
    do
    {
        if (!device_is_ready(app_shell_config.shellDev))
        {
            LOG_ERR("Device not ready");
            errorCode = -ENODEV;
            break;
        }

    } while (0);

    return errorCode;
}