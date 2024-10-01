#include "stm32f4xx.h"

// Variables globales
uint8_t contador = 0;
uint8_t modo_binario = 1; // 1: binario, 0: décadas

void configurarGPIO(void) {
    // Habilitar el reloj de los puertos GPIOA y GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    
    // Configurar los pines PA0, PA1, PA2, PA3 como salida (LEDs)
    GPIOA->MODER |= (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6);
    
    // Configurar los pines PC13, PC14, PC15 como entrada (botones)
    GPIOC->MODER &= ~((3 << 26) | (3 << 28) | (3 << 30)); // Entradas

    // Configurar resistencias internas
    GPIOC->PUPDR |= (2 << 26); // Pull-down para PC13 (Botón 1)
    GPIOC->PUPDR |= (1 << 28); // Pull-up para PC14 (Botón 2)
    // PC15 usará una resistencia externa, no configuramos PUPDR
}

void actualizarLEDs(void) {
    if (modo_binario) {
        // Mostrar el contador en binario
        GPIOA->ODR = (GPIOA->ODR & 0xFFFFFFF0) | (contador & 0x0F);
    } else {
        // Mostrar el contador en décadas
        uint8_t decenas = contador % 10;
        GPIOA->ODR = (GPIOA->ODR & 0xFFFFFFF0) | (decenas & 0x0F);
    }
}

void debounceDelay(void) {
    for (volatile int i = 0; i < 100000; i++);
}

int main(void) {
    // Configuración de GPIO
    configurarGPIO();

    while (1) {
        // Leer el estado del botón para cambiar de modo
        if (!(GPIOC->IDR & (1 << 13))) {
            debounceDelay();
            modo_binario = !modo_binario; // Cambiar el modo
        }

        // Leer el estado del botón para incrementar el contador
        if (!(GPIOC->IDR & (1 << 14))) {
            debounceDelay();
            if (contador < 15) contador++;
            actualizarLEDs();
        }

        // Leer el estado del botón para decrementar el contador
        if (!(GPIOC->IDR & (1 << 15))) {
            debounceDelay();
            if (contador > 0) contador--;
            actualizarLEDs();
        }
    }
}

