/*
 * main.c
 * EmbarcaTechRes U1 Ex04: Estação de Alerta de Enchente
 * Gabriel Marques
 * Implementação com FreeRTOS: tarefas, filas e comunicação
 */

#include <stdio.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

// ----- Configurações de hardware -----
#define JOYSTICK_ADC_WATER 0 // ADC0 para nível da água
#define JOYSTICK_ADC_RAIN 1  // ADC1 para volume de chuva

#define LED_RED_PIN 13
#define LED_GREEN_PIN 11
#define LED_BLUE_PIN 12

// Percentual desejado
#define PERCENT_WATER 70
#define PERCENT_RAIN  80

// Conversão para valor ADC
#define THRESHOLD_WATER  ((PERCENT_WATER * 4095) / 100)  // ≈ 2866
#define THRESHOLD_RAIN   ((PERCENT_RAIN  * 4095) / 100)  // ≈ 3276

// ----- Tipos de dados para filas -----
typedef struct
{
    uint16_t water_level; // 0-1023
    uint16_t rain_vol;    // 0-1023
} SensorData_t;

typedef enum
{
    MODE_NORMAL = 0,
    MODE_ALERT
} SystemMode_t;

typedef struct
{
    SystemMode_t mode;
    SensorData_t data;
} Command_t;

// ----- Handles de filas -----
static QueueHandle_t xSensorQueue;
static QueueHandle_t xCommandQueue;

// ----- Prototipação das tarefas -----
void vTaskJoystickRead(void *pvParameters);
void vTaskLogic(void *pvParameters);
void vTaskRGBLED(void *pvParameters);
void vTaskBuzzer(void *pvParameters);
/*void vTaskMatrix(void *pvParameters);
void vTaskDisplay(void *pvParameters);*/

int main()
{
    // Inicialização padrão
    stdio_init_all();
    adc_init();
    adc_gpio_init(26 + JOYSTICK_ADC_WATER); // PIO ADC0
    adc_gpio_init(26 + JOYSTICK_ADC_RAIN);

    // Configura pinos de saída
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);

    // Cria filas
    xSensorQueue = xQueueCreate(4, sizeof(SensorData_t));
    xCommandQueue = xQueueCreate(4, sizeof(Command_t));

    // Criar tarefas
    xTaskCreate(vTaskJoystickRead, "Joystick Read", 256, NULL, 2, NULL);
    xTaskCreate(vTaskLogic, "Logic", 512, NULL, 3, NULL);
    xTaskCreate(vTaskRGBLED, "Leds RGB", 256, NULL, 1, NULL);
    xTaskCreate(vTaskBuzzer, "Buzzer", 256, NULL, 1, NULL);
 /*   xTaskCreate(vTaskMatrix, "Matriz", 512, NULL, 1, NULL);
    xTaskCreate(vTaskDisplay, "Display", 512, NULL, 2, NULL);*/

    // Inicia scheduler
    vTaskStartScheduler();

    panic_unsupported();

    return 0;
}

// ==================== Implementações das tarefas ====================

void vTaskJoystickRead(void *pvParameters)
{
    SensorData_t sensor;
    while (true)
    {
        // Seleciona ADC e mede joystick
        adc_select_input(JOYSTICK_ADC_WATER);
        sensor.water_level = adc_read();

        printf("%d",sensor.water_level);
        adc_select_input(JOYSTICK_ADC_RAIN);
        sensor.rain_vol = adc_read();
         printf("%d",sensor.rain_vol);

        // Envia dados para a lógica
        xQueueOverwrite(xSensorQueue, &sensor);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void vTaskLogic(void *pvParameters)
{
    SensorData_t sensor;
    Command_t cmd;

    while (true)
    {
        if (xQueueReceive(xSensorQueue, &sensor, portMAX_DELAY) == pdTRUE)
        {
            // Determina modo
            if (sensor.water_level >= THRESHOLD_WATER || sensor.rain_vol >= THRESHOLD_RAIN)
            {
                cmd.mode = MODE_ALERT;
            }
            else
            {
                cmd.mode = MODE_NORMAL;
            }
            cmd.data = sensor;
            // Envia comando aos periféricos
            xQueueOverwrite(xCommandQueue, &cmd);
        }
    }
}

void vTaskRGBLED(void *pvParameters)
{
    Command_t cmd;
    while (true)
    {
        if (xQueueReceive(xCommandQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            if (cmd.mode == MODE_ALERT)
            {
                // Alerta: vermelho
                gpio_put(LED_RED_PIN, 1);
                gpio_put(LED_GREEN_PIN, 0);
                gpio_put(LED_BLUE_PIN, 0);
            }
            else
            {
                // Normal: verde
                gpio_put(LED_RED_PIN, 0);
                gpio_put(LED_GREEN_PIN, 1);
                gpio_put(LED_BLUE_PIN, 0);
            }
        }
    }
}

void vTaskBuzzer(void *pvParameters)
{
    Command_t cmd;
    while (true)
    {
        if (xQueueReceive(xCommandQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            if (cmd.mode == MODE_ALERT)
            {
                // Beep sonoro periódico
            }
            else
            {
                // Normal: buzzer desativado
            }
        }
    }
}
/*
void vTaskMatrix(void *pvParameters)
{
    // Recebe cmd via fila, desenha ícone na matriz
}

void vTaskDisplay(void *pvParameters)
{
    // Recebe cmd via fila, atualiza OLED com valores e destaque
}
*/