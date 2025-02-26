#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

// Definição de pinos
#define PIN_DHT22 2          // Pino para o sensor DHT22 (temperatura e umidade)
#define PIN_SOIL_MOISTURE 26 // Pino ADC0 (GP26) para sensor de umidade do solo
#define PIN_LIGHT_SENSOR 27  // Pino ADC1 (GP27) para sensor de luz
#define PIN_LED_ALERT 15     // LED para alertas

// Pinos para o LCD I2C
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define LCD_ADDR 0x27 // Endereço I2C padrão para LCDs

// Variáveis globais
float temperature = 0.0;
float humidity = 0.0;
int soil_moisture = 0;
int light_level = 0;

// Limites para alertas
const float TEMP_MIN = 15.0;
const float TEMP_MAX = 30.0;
const int SOIL_MIN = 500;   // Valores de 0-4095 para ADC do Pico
const int LIGHT_MIN = 1000; // Valores de 0-4095 para ADC do Pico

// Protótipos de funções
void init_sensors(void);
void read_sensors(void);
void process_data(void);
void display_data(void);
void check_alerts(void);
void lcd_init(void);
void lcd_print(const char *message, int row);

int main()
{
    // Inicialização do sistema
    stdio_init_all();
    init_sensors();
    lcd_init();

    // LED de alerta
    gpio_init(PIN_LED_ALERT);
    gpio_set_dir(PIN_LED_ALERT, GPIO_OUT);

    printf("Sistema de Monitoramento Ambiental Iniciado\n");
    lcd_print("Sistema Iniciado", 0);
    sleep_ms(2000);

    while (true)
    {
        read_sensors();
        process_data();
        display_data();
        check_alerts();

        // Intervalo entre leituras
        sleep_ms(10000); // 10 segundos
    }

    return 0;
}

void init_sensors(void)
{
    // Inicialização do ADC
    adc_init();
    adc_gpio_init(PIN_SOIL_MOISTURE);
    adc_gpio_init(PIN_LIGHT_SENSOR);

    // Inicialização do pino DHT22
    gpio_init(PIN_DHT22);

    // Inicialização do I2C para o LCD
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

void read_sensors(void)
{
    // Leitura do sensor de umidade do solo
    adc_select_input(0); // ADC0 corresponde ao GP26
    soil_moisture = adc_read();

    // Leitura do sensor de luz
    adc_select_input(1); // ADC1 corresponde ao GP27
    light_level = adc_read();

    // Simulação de leitura do DHT22 (em uma implementação real, usaria uma biblioteca específica)
    // Na prática, você precisaria implementar o protocolo de comunicação do DHT22 ou usar uma biblioteca
    temperature = 25.0; // Valores simulados
    humidity = 60.0;    // Valores simulados

    printf("Leitura de sensores: Temp=%.1f°C, Umid=%.1f%%, Solo=%d, Luz=%d\n",
           temperature, humidity, soil_moisture, light_level);
}

void process_data(void)
{
    // Aqui poderia haver algum processamento adicional dos dados
    // Por exemplo, média de leituras ou conversão dos valores brutos

    // Convertendo valores ADC para percentuais (exemplo)
    soil_moisture = (4095 - soil_moisture) * 100 / 4095; // Inverte a leitura, pois geralmente menor valor = mais úmido
    light_level = light_level * 100 / 4095;
}

void display_data(void)
{
    // Exibição no terminal serial
    printf("Dados processados:\n");
    printf("Temperatura: %.1f°C\n", temperature);
    printf("Umidade: %.1f%%\n", humidity);
    printf("Umidade do solo: %d%%\n", soil_moisture);
    printf("Nível de luz: %d%%\n", light_level);

    // Exibição no LCD
    char buf[17];
    snprintf(buf, sizeof(buf), "Temp: %.1fC %d%%", temperature, (int)humidity);
    lcd_print(buf, 0);

    snprintf(buf, sizeof(buf), "Solo: %d%% Luz: %d%%", soil_moisture, light_level);
    lcd_print(buf, 1);
}

void check_alerts(void)
{
    bool alert = false;

    if (temperature < TEMP_MIN || temperature > TEMP_MAX)
    {
        printf("ALERTA: Temperatura fora da faixa ideal!\n");
        alert = true;
    }

    if (soil_moisture < SOIL_MIN)
    {
        printf("ALERTA: Solo muito seco!\n");
        alert = true;
    }

    if (light_level < LIGHT_MIN)
    {
        printf("ALERTA: Pouca luz!\n");
        alert = true;
    }

    // Ativa LED de alerta se necessário
    gpio_put(PIN_LED_ALERT, alert);
}

void lcd_init(void)
{
    // Esta é uma função esqueleto - na implementação real, você precisaria
    // usar uma biblioteca para comunicação I2C com o LCD ou implementar o protocolo
    printf("LCD inicializado\n");
}

void lcd_print(const char *message, int row)
{
    // Esta é uma função esqueleto - na implementação real, você enviaria
    // os comandos apropriados via I2C para o LCD
    printf("LCD[%d]: %s\n", row, message);
}