#include <stdint.h> 

#define LED_ADDR 0x00002000
#define LED_REG *((volatile uint32_t*)LED_ADDR)

// Simple delay function to make the LEDs visible to the human eye
void delay(uint32_t count) {
    for (volatile uint32_t i = 0; i < count; i++) {
        // __asm__ volatile ("nop") prevents the compiler from optimizing the loop away
        __asm__ volatile ("nop"); 
    }
}

// int main () {
//     uint32_t counter = 0; 

//     // Infinite loop to prove continuous AXI instruction fetching from DDR3
//     while (1) {
//         LED_REG = counter; 
//         counter++; 
        
//         // Adjust this number if the LEDs blink too fast or too slow.
//         // 1,000,000 is a good starting point for a ~50-100 MHz CPU.
//         delay(1000000); 
//     }

//     return 0;
// }


int main () {
    // STAGE 1: Test Load Byte Unsigned (lbu) vs Signed (lb)
    volatile int8_t  s_byte = (int8_t)0x80;   // Holds 0x80 (-128)
    volatile uint8_t u_byte = (uint8_t)0x80;  // Holds 0x80 (+128)

    volatile int32_t  lb_res  = s_byte;   // Triggers 'lb'  -> 0xFFFFFF80 (-128)
    volatile uint32_t lbu_res = u_byte;  // Triggers 'lbu' -> 0x00000080 (+128)

    if (lb_res != -128 || lbu_res != 128) {
        LED_REG = 0b1001; // FAIL Stage 1 (LED 9)
        while(1);
    }
    LED_REG = 1; // PASS Stage 1 (LED 1)
    delay(2000000);

    // STAGE 2: Test Load Halfword Signed (lh) vs Unsigned (lhu)
    volatile int16_t  s_half = (int16_t)0x8000;   // Holds 0x8000 (-32768)
    volatile uint16_t u_half = (uint16_t)0x8000;  // Holds 0x8000 (+32768)

    volatile int32_t  lh_res  = s_half;   // Triggers 'lh'  -> 0xFFFF8000 (-32768)
    volatile uint32_t lhu_res = u_half;  // Triggers 'lhu' -> 0x00008000 (+32768)

    if (lh_res != -32768 || lhu_res != 32768) {
        LED_REG = 0b1010; // FAIL Stage 2 (LED 10)
        while(1);
    }
    LED_REG = 2; // PASS Stage 2 (LED 2)
    delay(2000000);

    // STAGE 3: Test RV32M - Multiplication (mul)
    volatile int32_t a_mul = -15;
    volatile int32_t b_mul = 20;
    volatile int32_t mul_res = a_mul * b_mul; // Triggers 'mul' -> -300

    if (mul_res != -300) {
        LED_REG = 0b1011; // FAIL Stage 3 (LED 11)
        while(1);
    }
    LED_REG = 3; // PASS Stage 3 (LED 3)
    delay(2000000);

    // STAGE 4: Test RV32M - Division & Remainder (div, rem, divu, remu)
    volatile int32_t s_num = -100;
    volatile int32_t s_den = 7;
    volatile int32_t div_res = s_num / s_den; // Triggers 'div' -> -14
    volatile int32_t rem_res = s_num % s_den; // Triggers 'rem' -> -2

    volatile uint32_t u_num = 100;
    volatile uint32_t u_den = 7;
    volatile uint32_t divu_res = u_num / u_den; // Triggers 'divu' -> 14
    volatile uint32_t remu_res = u_num % u_den; // Triggers 'remu' -> 2

    if (div_res != -14 || rem_res != -2 || divu_res != 14 || remu_res != 2) {
        LED_REG = 0b1100; // FAIL Stage 4 (LED 12)
        while(1);
    }

    // ALL TESTS PASSED! Show 15 (0b1111) on LEDs
    while(1) {
        LED_REG = 15; 
    }

    return 0;
}