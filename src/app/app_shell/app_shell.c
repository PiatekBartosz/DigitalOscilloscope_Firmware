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

static int app_shell_fooCmd(const struct shell *sh, size_t argc,
				   char **argv)
{
    return 0;
	// return cmd_log_test_start(sh, argc, argv, 200);
}

int app_shell_init(void)
{
    return 0;
}
