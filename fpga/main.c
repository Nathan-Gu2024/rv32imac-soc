#include <stdint.h> 

#define LED_ADDR 0x20000000 
#define LED_REG  *((volatile uint32_t*)LED_ADDR)

int main () {
    volatile uint32_t payload = 5; 

    LED_REG = payload; 

    return 0;
}