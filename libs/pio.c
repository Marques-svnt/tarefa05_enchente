#include <stdio.h>
#include <stdlib.h>

// Bibliotecas de hardware do Raspberry Pi Pico
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

// Headers do projeto
#include "pio.h"
#include "ws2812.pio.h"

#define NUM_PIXELS 25
#define WS2812_PIN 7
#define IS_RGBW false


/* Buffer para armazenar quais LEDs estão ligados matriz 5x5
(obs: devido a organização do diagrama, teremos que configurar as matrizes "espelhadas")*/
bool led_buffer[4][NUM_PIXELS] = {
    // Seta
    {0, 0, 1, 0, 0, 
    0, 0, 1, 0, 0, 
    1, 0, 1, 0, 1, 
    0, 1, 1, 1, 0, 
    0, 0, 1, 0, 0,},
    // O
    {0, 1, 1, 1, 0, 
    0, 1, 0, 1, 0, 
    0, 1, 0, 1, 0, 
    0, 1, 0, 1, 0, 
    0, 1, 1, 1, 0,},
    // X
    {0, 1, 0, 1, 0, 
    0, 1, 0, 1, 0, 
    0, 0, 1, 0, 0, 
    0, 1, 0, 1, 0, 
    0, 1, 0, 1, 0,},
    // Off
    {0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0,}

};

static inline void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

void set_one_led(int index, uint8_t r, uint8_t g, uint8_t b)
{
    // Define a cor com base nos parâmetros fornecidos
    uint32_t color = urgb_u32(r, g, b);

    // Define todos os LEDs com a cor especificada
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        if (led_buffer[index][i])
        {
            put_pixel(color); // Liga o LED com um no buffer
        }
        else
        {
            put_pixel(0); // Desliga os LEDs com zero no buffer
        }
    }
}

// Função que vai inicializar o PIO no microcontrolador
void initializePio()
{
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
}
