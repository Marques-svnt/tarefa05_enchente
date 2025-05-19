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
#include "hardware/pwm.h"

#include "buzzer.h"
#include "pio.h"
#include "display.h"

// ----- Configurações de hardware -----
#define JOYSTICK_ADC_WATER 0 // ADC0 para nível da água
#define JOYSTICK_ADC_RAIN 1  // ADC1 para volume de chuva

#define LED_RED_PIN 13
#define LED_GREEN_PIN 11
#define LED_BLUE_PIN 12

// Percentual desejado
#define PERCENT_WATER 70
#define PERCENT_RAIN 80

// Conversão para valor ADC
#define THRESHOLD_WATER ((PERCENT_WATER * 4095) / 100) // ≈ 2866
#define THRESHOLD_RAIN ((PERCENT_RAIN * 4095) / 100)   // ≈ 3276

// PWM slices para LEDs
static uint slice_red, slice_green, slice_blue;

// Variável global para guardar o slice do PWM
static uint pwm_slice;

// ----- Tipos de dados para filas -----
typedef struct
{
    uint16_t water_level; // 0-1023
    uint16_t rain_vol;    // 0-1023
} SensorData_t;

typedef enum
{
    MODE_NORMAL = 0,
    MODE_WARNING,
    MODE_ALERT
} SystemMode_t;

typedef struct
{
    SystemMode_t mode;
    SensorData_t data;
    bool alert; // true = tocar, false = parar
    uint freq;  // frequência em Hz (quando alert=true)
} Command_t;

// ----- Handles de filas -----
static QueueHandle_t xSensorQueue;
static QueueHandle_t xCommandQueue;
static volatile Command_t lastCommand;

// ----- Prototipação das tarefas -----
void vTaskJoystickRead(void *pvParameters);
void vTaskLogic(void *pvParameters);
void vTaskRGBLED(void *pvParameters);
void vTaskBuzzer(void *pvParameters);
void vTaskMatrix(void *pvParameters);
void vTaskDisplay(void *pvParameters);

int main()
{
    // Inicialização padrão
    stdio_init_all();
    adc_init();
    pwm_slice = buzzer_init();

    adc_gpio_init(26 + JOYSTICK_ADC_WATER); // PIO ADC0
    adc_gpio_init(26 + JOYSTICK_ADC_RAIN);

    // Configura PWM para LEDs
    gpio_set_function(LED_RED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_GREEN_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_BLUE_PIN, GPIO_FUNC_PWM);

    slice_red = pwm_gpio_to_slice_num(LED_RED_PIN);
    slice_green = pwm_gpio_to_slice_num(LED_GREEN_PIN);
    slice_blue = pwm_gpio_to_slice_num(LED_BLUE_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);

    pwm_init(slice_red, &config, true);
    pwm_init(slice_green, &config, true);
    pwm_init(slice_blue, &config, true);

    // Cria filas
    xSensorQueue = xQueueCreate(4, sizeof(SensorData_t));
    xCommandQueue = xQueueCreate(1, sizeof(Command_t));

    // Criar tarefas
    xTaskCreate(vTaskJoystickRead, "Joystick Read", 256, NULL, 2, NULL);
    xTaskCreate(vTaskLogic, "Logic", 512, NULL, 3, NULL);
    xTaskCreate(vTaskRGBLED, "Leds RGB", 256, NULL, 1, NULL);
    xTaskCreate(vTaskBuzzer, "Buzzer", 256, NULL, 1, NULL);
    xTaskCreate(vTaskMatrix, "Matriz", 512, NULL, 1, NULL);
    xTaskCreate(vTaskDisplay, "Display", 512, NULL, 1, NULL);

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

        printf("%d", sensor.water_level);
        adc_select_input(JOYSTICK_ADC_RAIN);
        sensor.rain_vol = adc_read();
        printf("%d", sensor.rain_vol);

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
                cmd.alert = true;
                cmd.freq = 1000;
            }
            else if ((sensor.water_level >= (THRESHOLD_WATER - 200) || sensor.rain_vol >= (THRESHOLD_RAIN - 200)))
            {
                cmd.mode = MODE_WARNING;
                cmd.alert = false;
                cmd.freq = 0;
            }
            else
            {
                cmd.mode = MODE_NORMAL;
                cmd.alert = false;
                cmd.freq = 0;
            }
            cmd.data = sensor;
            lastCommand = cmd;
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
        if (xQueueReceive(xCommandQueue, &cmd, portMAX_DELAY))
        {
            uint16_t level = cmd.data.water_level; // ou média dos dois sensores
            // verde varia com nivel em modo normal
            if (cmd.mode == MODE_NORMAL)
            {
                uint16_t duty = level * 65535 / 4095;
                pwm_set_chan_level(slice_green, pwm_gpio_to_channel(LED_GREEN_PIN), duty);
                pwm_set_chan_level(slice_red, pwm_gpio_to_channel(LED_RED_PIN), 0);
            }
            // warning: amarelo (vermelho+verde)
            else if (cmd.mode == MODE_WARNING)
            {
                pwm_set_chan_level(slice_red, pwm_gpio_to_channel(LED_RED_PIN), 32767);
                pwm_set_chan_level(slice_green, pwm_gpio_to_channel(LED_GREEN_PIN), 32767);
            }
            // alerta: vermelho
            else
            {
                pwm_set_chan_level(slice_red, pwm_gpio_to_channel(LED_RED_PIN), 65535);
                pwm_set_chan_level(slice_green, pwm_gpio_to_channel(LED_GREEN_PIN), 0);
            }
            // azul fica desligado
            pwm_set_chan_level(slice_blue, pwm_gpio_to_channel(LED_BLUE_PIN), 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTaskBuzzer(void *pvParameters)
{
    Command_t cmd;
    while (true)
    {
        // Espera pelo próximo comando de buzzer
        if (xQueueReceive(xCommandQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            if (cmd.alert)
            {
                // toca no tom especificado
                buzz(cmd.freq);
            }
            else
            {
                // desliga o buzzer
                buzzer_stop();
            }
        }
    }
}

void vTaskMatrix(void *pvParameters)
{
    Command_t cmd;

    initializePio();

    while (true)
    {
        if (xQueueReceive(xCommandQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            int pattern;
            uint8_t r, g, b;

            switch (cmd.mode)
            {
            case MODE_NORMAL:
                pattern = 0;
                r = 0;
                g = 64;
                b = 0;
                break;
            case MODE_WARNING:
                pattern = 0;
                r = 64;
                g = 64;
                b = 0;
                break;
            case MODE_ALERT:
            default:
                pattern = 1;
                r = 64;
                g = 0;
                b = 0;
                break;
            }

            set_one_led(pattern, r, g, b);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTaskDisplay(void *pvParameters)
{
    Command_t cmd;
    char buf[20];

    initI2C();

    while (true)
    {
        if (xQueueReceive(xCommandQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            limpar();

            draw_rect();

            uint16_t water_percent = cmd.data.water_level * 100 / 4095;
            snprintf(buf, sizeof(buf), "Water: %3u%%%", water_percent);
            draw_text(buf, 16, 12);

            uint16_t rain_percent = cmd.data.rain_vol * 100 / 4095;
            snprintf(buf, sizeof(buf), "Rain:  %3u%%%", rain_percent);
            draw_text(buf, 16, 28);

            if (cmd.mode == MODE_ALERT)
                draw_text("() ALERTA ()", 20, 48);
            else
                draw_text("NORMAL", 40, 48);

            flush_display();

            // pequena pausa para não saturar o barramento I2C
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
