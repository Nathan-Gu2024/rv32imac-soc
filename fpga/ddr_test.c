#include <stdint.h> 

#define LED_ADDR 0x00002000
#define LED_REG  *((volatile uint32_t*)LED_ADDR)

// Simple delay function to make the LEDs visible to the human eye
void delay(uint32_t count) {
    for (volatile uint32_t i = 0; i < count; i++) {
        // __asm__ volatile ("nop") prevents the compiler from optimizing the loop away
        __asm__ volatile ("nop"); 
    }
}

int main () {
    uint32_t counter = 0; 

    // Infinite loop to prove continuous AXI instruction fetching from DDR3
    while (1) {
        LED_REG = counter; 
        counter++; 
        
        // Adjust this number if the LEDs blink too fast or too slow.
        // 1,000,000 is a good starting point for a ~50-100 MHz CPU.
        delay(1000000); 
    }

    return 0;
}