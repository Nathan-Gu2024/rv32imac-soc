#include <stdint.h> 

#define LED_ADDR 0x00002000
#define LED_REG *((volatile uint32_t*)LED_ADDR)

// int main () {
//     volatile uint32_t payload = 5; 

//     LED_REG = payload; 

//     return 0;
// }

// A simple delay function to stall the CPU
// void delay(uint32_t count) {
//     // 'volatile' prevents GCC from optimizing this loop away
//     for (volatile uint32_t i = 0; i < count; i++); 
// }

// int main () {
//     uint32_t counter = 0; 

//     while (1) {
//         LED_REG = counter; 
//         counter++; 
        
//         // Adjust this number based on your FPGA clock speed (e.g., 50MHz)
//         // If it blinks too fast to see, add a zero!
//         delay(500000); 
//     }

//     return 0;
// }

// void delay(uint32_t count) {
//     for (volatile uint32_t i = 0; i < count; i++); 
// }

// int main () {
//     uint32_t a = 0;
//     uint32_t b = 1;
//     uint32_t next;
    
//     while (1) {
//         LED_REG = a; 
        
//         next = a + b; // Heavy dependency here!
//         a = b;
//         b = next;
        
//         delay(2000000);
        
//         // Reset so we don't overflow the LEDs visually
//         // Assuming you have 4-8 LEDs (max value ~15 to 255)
//         if (a > 128) { 
//             a = 0;
//             b = 1;
//         }
//     }
//     return 0;
// }

// void delay(uint32_t count) {
//     for (volatile uint32_t i = 0; i < count; i++); 
// }

// int main () {
//     // Store values in TCM (Data Memory)
//     volatile uint32_t pattern[4] = {1, 2, 4, 8}; 
    
//     while (1) {
//         for (int i = 0; i < 4; i++) {
//             // lw from TCM, then sw to MMIO
//             LED_REG = pattern[i]; 
//             delay(1000000);
//         }
//     }
//     return 0;
// }

// // 1. Tests Structs (Memory with non-zero offsets)
// typedef struct {
//     uint32_t current_val;
//     uint32_t increment;
//     uint32_t (*math_op)(uint32_t); // 2. Tests jalr (indirect jump)
// } Task;

// // 3. Tests Stack Depth, Register Saving, and jalr for 'ret'
// uint32_t recursive_sum(uint32_t n) {
//     if (n <= 1) {
//         return 1;
//     }
//     // Forces the compiler to push 'ra' and 'n' to the stack, 
//     // decrement the stack pointer (sp), and pop them later.
//     return n + recursive_sum(n - 1);
// }

// void delay(uint32_t count) {
//     for (volatile uint32_t i = 0; i < count; i++); 
// }

// int main () {
//     // Initialize the struct on the stack
//     Task myTask;
//     myTask.current_val = 1;
//     myTask.increment = 1;
//     myTask.math_op = recursive_sum; 
    
//     while (1) {
//         // Reads struct members (lw with non-zero offsets)
//         // Jumps to the function pointer (jalr)
//         uint32_t result = myTask.math_op(myTask.current_val);
        
//         LED_REG = result; 
        
//         myTask.current_val += myTask.increment;
        
//         // Reset after 5 to keep the LED output visible (1, 3, 6, 10, 15)
//         if (myTask.current_val > 5) {
//             myTask.current_val = 1;
//         }
        
//         delay(2000000); 
//     }

//     return 0;
// }


// void delay(uint32_t count) {
//     for (volatile uint32_t i = 0; i < count; i++); 
// }

// int main () {
//     // 1. Initialize a full 32-bit word to zero
//     volatile uint32_t test_word = 0x00000000;
    
//     // 2. Test Store Byte (sb) and d_wmask
//     volatile uint8_t* byte_ptr = (volatile uint8_t*)&test_word;
//     byte_ptr[0] = 0xAA;
//     byte_ptr[1] = 0xBB;
//     byte_ptr[2] = 0xCC;
//     byte_ptr[3] = 0xDD;
    
//     // RISC-V is Little Endian, so bytes are stored backwards in the word
//     // If d_wmask works, the word is now exactly 0xDDCCBBAA
//     if (test_word == 0xDDCCBBAA) {
//         LED_REG = 1; // Stage 1 Pass (LED: 001)
//     } else {
//         LED_REG = 7; // FAIL (LED: 111)
//         while(1);
//     }
//     delay(2000000);

//     // 3. Test Store Halfword (sh)
//     volatile uint16_t* half_ptr = (volatile uint16_t*)&test_word;
//     half_ptr[0] = 0x1234;
//     half_ptr[1] = 0x5678;
    
//     // If halfword masks work, the word is now 0x56781234
//     if (test_word == 0x56781234) {
//         LED_REG = 2; // Stage 2 Pass (LED: 010)
//     } else {
//         LED_REG = 7; 
//         while(1);
//     }
//     delay(2000000);

//     // 4. Test Sign Extension on Loads (lb vs lbu)
//     volatile int8_t negative_byte = -5;  // Memory holds 0xFB
//     volatile int32_t sign_extended = negative_byte; // CPU executes 'lb'
    
//     // If 'lb' correctly sign-extends, it pads 1s (0xFFFFFFFB)
//     // If it incorrectly uses 'lbu', it pads 0s (0x000000FB), failing this check.
//     if (sign_extended == -5) {
//         LED_REG = 3; // Stage 3 Pass (LED: 011)
//     } else {
//         LED_REG = 7;
//         while(1);
//     }

//     // Success loop! Stay on 3.
//     while(1) {
//         LED_REG = 3; 
//     }

//     return 0;
// }

void delay(uint32_t count) {
    for (volatile uint32_t i = 0; i < count; i++); 
}

int main () {
    // =========================================================
    // STAGE 1: Test Load Byte Unsigned (lbu) vs Signed (lb)
    // =========================================================
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

    // =========================================================
    // STAGE 2: Test Load Halfword Signed (lh) vs Unsigned (lhu)
    // =========================================================
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

    // =========================================================
    // STAGE 3: Test RV32M - Multiplication (mul)
    // =========================================================
    volatile int32_t a_mul = -15;
    volatile int32_t b_mul = 20;
    volatile int32_t mul_res = a_mul * b_mul; // Triggers 'mul' -> -300

    if (mul_res != -300) {
        LED_REG = 0b1011; // FAIL Stage 3 (LED 11)
        while(1);
    }
    LED_REG = 3; // PASS Stage 3 (LED 3)
    delay(2000000);

    // =========================================================
    // STAGE 4: Test RV32M - Division & Remainder (div, rem, divu, remu)
    // =========================================================
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

    // =========================================================
    // ALL TESTS PASSED! Show 15 (0b1111) on LEDs
    // =========================================================
    while(1) {
        LED_REG = 15; 
    }

    return 0;
}