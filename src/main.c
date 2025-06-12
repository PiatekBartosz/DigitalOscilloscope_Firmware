#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(main); 

int main(void)
{
    int i = 10;

    while(1)
    {
       printk("Test %d \n", ++i);
       k_sleep(K_SECONDS(1));
    }

    return 0;
}