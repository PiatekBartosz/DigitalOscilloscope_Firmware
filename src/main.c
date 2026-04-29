#include "app/app.h"
#include <zephyr/kernel.h>

/* Private Macros */

/* Private Types */

/* Private Variables */

/* Private Function Prototypes */

/* Private Functions */

/* Public Functions */

int main(void)
{
    printk("Hello world from oscilloscope project!\n");

    int ret = app_init();

    size_t counter = 0u;
    while (1)
    {
        printk("Fatal error %d. Waiting for reboot... %zu\n", ret, counter++);
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
