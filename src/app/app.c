#include "app.h"
#include "afe_manager/afe_manager.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

/* Private Macros */

LOG_MODULE_REGISTER(app);

/* Private Types */

/* Private Variables */

/* Private Function Prototypes */

/* Private Functions */

/* Public Functions */

int app_init(void)
{
    int errorCode = 0;

    do
    {
        errorCode = afe_manager_init();
        if (errorCode != 0)
        {
            LOG_ERR("Error initalizing afe manager: %d", errorCode);
        }

        while (true)
        {
            k_sleep(K_SECONDS(1));
        }

    } while (0);

    return errorCode;
}
