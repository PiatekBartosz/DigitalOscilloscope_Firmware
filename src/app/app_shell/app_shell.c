// TODO: do template

#include <app_shell/app_shell.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(app_shell);

typedef struct app_shell_config_s
{
    const struct device *shellDev;

} app_shell_config_t;

static app_shell_config_t app_shell_config = 
{
    .shellDev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart)), 
};

static void hw_set_gain(uint8_t ch, int gain)
{
    printk("HW: Set CH%d gain to %d\n", ch, gain);
}

static void hw_set_offset(uint8_t ch, int offset)
{
    printk("HW: Set CH%d offset to %d\n", ch, offset);
}

static void hw_set_coupling(uint8_t ch, bool ac)
{
    printk("HW: Set CH%d coupling to %s\n", ch, ac ? "AC" : "DC");
}

static void hw_set_cso(bool enable)
{
    printk("HW: CSO %s\n", enable ? "ON" : "OFF");
}

/* ========================= */
/*     Shell Commands        */
/* ========================= */

static bool parse_channel(const char *arg, uint8_t *ch)
{
    int val = atoi(arg);

    if (val < 1 || val > 2)
    {
        return false;
    }

    *ch = (uint8_t)val;
    return true;
}

/* scope gain <ch> <value> */
static int cmd_scope_gain(const struct shell *sh, size_t argc, char **argv)
{
    uint8_t ch;

    if (!parse_channel(argv[1], &ch))
    {
        shell_error(sh, "Invalid channel. Use 1 or 2.");
        return -EINVAL;
    }

    int gain = atoi(argv[2]);

    hw_set_gain(ch, gain);

    shell_print(sh, "CH%d gain set to %d", ch, gain);
    return 0;
}

/* scope offset <ch> <value> */
static int cmd_scope_offset(const struct shell *sh, size_t argc, char **argv)
{
    uint8_t ch;

    if (!parse_channel(argv[1], &ch))
    {
        shell_error(sh, "Invalid channel. Use 1 or 2.");
        return -EINVAL;
    }

    int offset = atoi(argv[2]);

    hw_set_offset(ch, offset);

    shell_print(sh, "CH%d offset set to %d", ch, offset);
    return 0;
}

/* scope coupling <ch> <ac|dc> */
static int cmd_scope_coupling(const struct shell *sh, size_t argc, char **argv)
{
    uint8_t ch;

    if (!parse_channel(argv[1], &ch))
    {
        shell_error(sh, "Invalid channel. Use 1 or 2.");
        return -EINVAL;
    }

    if (strcmp(argv[2], "ac") == 0)
    {
        hw_set_coupling(ch, true);
        shell_print(sh, "CH%d coupling set to AC", ch);
    }
    else if (strcmp(argv[2], "dc") == 0)
    {
        hw_set_coupling(ch, false);
        shell_print(sh, "CH%d coupling set to DC", ch);
    }
    else
    {
        shell_error(sh, "Invalid mode. Use 'ac' or 'dc'.");
        return -EINVAL;
    }

    return 0;
}

/* scope cso <on|off> */
static int cmd_scope_cso(const struct shell *sh, size_t argc, char **argv)
{
    if (strcmp(argv[1], "on") == 0)
    {
        hw_set_cso(true);
        shell_print(sh, "CSO enabled");
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        hw_set_cso(false);
        shell_print(sh, "CSO disabled");
    }
    else
    {
        shell_error(sh, "Invalid option. Use 'on' or 'off'.");
        return -EINVAL;
    }

    return 0;
}

/* ========================= */
/*    Subcommand Structure   */
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
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(scope, &sub_scope,
                   "Digital oscilloscope control commands",
                   NULL);

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
