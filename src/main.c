#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(main); 

int main(void)
{
    printk("Hello world from oscilocope project!\n");

    while(1)
    {
        k_sleep(K_SECONDS(10));
    }

    return 0;
}