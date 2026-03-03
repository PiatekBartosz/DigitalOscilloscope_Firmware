#include <zephyr/kernel.h>
#include "app/app.h"

/* Private Macros */

/* Private Types */

/* Private Variables */

/* Private Function Prototypes */

/* Private Functions */

/* Public Functions */

int main(void)
{
    printk("Hello world from oscilocope project!\n");

    app_init();

    size_t counter = 0u;
    while (1)
    {
        printk("Fatal error. Waiting for reboot... %u", counter++);
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
