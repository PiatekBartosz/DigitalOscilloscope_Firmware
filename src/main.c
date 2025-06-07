// #include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

// LOG_MODULE_REGISTER(main); 

int main(void)
{
    int i = 0;

    while(1)
    {
       printk("Test %d \n", ++i);
    }

    return 0;
}