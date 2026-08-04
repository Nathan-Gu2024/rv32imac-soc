#include <stdint.h>

#define UART_TX_DATA   *((volatile uint32_t*)0x40001000)
#define UART_TX_STATUS *((volatile uint32_t*)0x40001004)

void uart_putchar(char c) {
    // Wait until the UART is ready (Status == 1)
    while (UART_TX_STATUS == 0) {
        // Spin and wait
    }
    // Write the character to trigger transmission
    UART_TX_DATA = c;
}

void uart_print(const char* str) {
    while (*str) {
        uart_putchar(*str++);
    }
}

int main() {
    uart_print("Hello from a custom RISC-V CPU!\r\n");
    while (1);
    return 0;
}