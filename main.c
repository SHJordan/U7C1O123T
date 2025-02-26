#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    // Comandos de inicialização para LCD 16x2 com interface I2C
    uint8_t lcd_init_cmds[] = {
        0x33, // Inicialização em modo 8-bits (parte 1)
        0x32, // Inicialização em modo 8-bits (parte 2)
        0x28, // Configura para modo 4-bits, 2 linhas, fonte 5x8
        0x0C, // Display ligado, cursor desligado, sem piscar
        0x06, // Cursor move para direita, sem deslocamento do display
        0x01  // Limpa o display
    };

    // Espera mais de 40ms após ligar para o LCD estabilizar
    sleep_ms(50);

    // Envia comandos de inicialização
    printf("Inicializando LCD I2C no endereço 0x%02X...\n", LCD_ADDR);

    for (int i = 0; i < sizeof(lcd_init_cmds); i++)
    {
        // Envio de cada comando para o LCD
        uint8_t buf[4];

        // Primeiro nibble (4 bits mais significativos)
        buf[0] = 0x00 | (lcd_init_cmds[i] & 0xF0); // EN=0, RS=0, dados
        buf[1] = 0x04 | (lcd_init_cmds[i] & 0xF0); // EN=1, RS=0, dados
        buf[2] = 0x00 | (lcd_init_cmds[i] & 0xF0); // EN=0, RS=0, dados

        // Envia o primeiro nibble
        if (i2c_write_blocking(i2c0, LCD_ADDR, buf, 3, false) != 3)
        {
            printf("Erro na comunicação I2C (primeiro nibble)\n");
            return;
        }

        // Segundo nibble (4 bits menos significativos)
        buf[0] = 0x00 | ((lcd_init_cmds[i] << 4) & 0xF0); // EN=0, RS=0, dados
        buf[1] = 0x04 | ((lcd_init_cmds[i] << 4) & 0xF0); // EN=1, RS=0, dados
        buf[2] = 0x00 | ((lcd_init_cmds[i] << 4) & 0xF0); // EN=0, RS=0, dados

        // Envia o segundo nibble
        if (i2c_write_blocking(i2c0, LCD_ADDR, buf, 3, false) != 3)
        {
            printf("Erro na comunicação I2C (segundo nibble)\n");
            return;
        }

        // Espera tempo suficiente após cada comando
        if (lcd_init_cmds[i] == 0x01)
        {
            sleep_ms(2); // Comando de limpar display precisa de mais tempo
        }
        else
        {
            sleep_us(50); // Outros comandos precisam de pelo menos 37us
        }
    }

    // Define a backlight como ligada
    uint8_t backlight_on = 0x08; // Liga o backlight
    i2c_write_blocking(i2c0, LCD_ADDR, &backlight_on, 1, false);

    printf("LCD inicializado com sucesso!\n");
}

void lcd_print(const char *message, int row)
{
    uint8_t cursor_pos;

    // Define a posição do cursor baseado na linha (row)
    // Linha 0: Posição 0x00, Linha 1: Posição 0x40
    cursor_pos = row == 0 ? 0x80 : 0xC0;

    // Envia comando para posicionar o cursor
    uint8_t buf[4];

    // Primeiro nibble (4 bits mais significativos) do comando
    buf[0] = 0x00 | (cursor_pos & 0xF0); // EN=0, RS=0 (comando), dados
    buf[1] = 0x04 | (cursor_pos & 0xF0); // EN=1, RS=0 (comando), dados
    buf[2] = 0x00 | (cursor_pos & 0xF0); // EN=0, RS=0 (comando), dados

    if (i2c_write_blocking(i2c0, LCD_ADDR, buf, 3, false) != 3)
    {
        printf("Erro ao posicionar cursor (primeiro nibble)\n");
        return;
    }

    // Segundo nibble (4 bits menos significativos) do comando
    buf[0] = 0x00 | ((cursor_pos << 4) & 0xF0); // EN=0, RS=0 (comando), dados
    buf[1] = 0x04 | ((cursor_pos << 4) & 0xF0); // EN=1, RS=0 (comando), dados
    buf[2] = 0x00 | ((cursor_pos << 4) & 0xF0); // EN=0, RS=0 (comando), dados

    if (i2c_write_blocking(i2c0, LCD_ADDR, buf, 3, false) != 3)
    {
        printf("Erro ao posicionar cursor (segundo nibble)\n");
        return;
    }

    sleep_us(50); // Espera um pouco após enviar o comando

    // Envia os caracteres da mensagem um por um
    for (int i = 0; i < strlen(message) && i < 16; i++)
    {
        uint8_t c = message[i];

        // Primeiro nibble (4 bits mais significativos) do caractere
        buf[0] = 0x01 | (c & 0xF0); // EN=0, RS=1 (dados), dados
        buf[1] = 0x05 | (c & 0xF0); // EN=1, RS=1 (dados), dados
        buf[2] = 0x01 | (c & 0xF0); // EN=0, RS=1 (dados), dados

        if (i2c_write_blocking(i2c0, LCD_ADDR, buf, 3, false) != 3)
        {
            printf("Erro ao escrever caractere (primeiro nibble)\n");
            return;
        }

        // Segundo nibble (4 bits menos significativos) do caractere
        buf[0] = 0x01 | ((c << 4) & 0xF0); // EN=0, RS=1 (dados), dados
        buf[1] = 0x05 | ((c << 4) & 0xF0); // EN=1, RS=1 (dados), dados
        buf[2] = 0x01 | ((c << 4) & 0xF0); // EN=0, RS=1 (dados), dados

        if (i2c_write_blocking(i2c0, LCD_ADDR, buf, 3, false) != 3)
        {
            printf("Erro ao escrever caractere (segundo nibble)\n");
            return;
        }

        sleep_us(50); // Espera um pouco para o LCD processar
    }

    // Adiciona o bit de backlight a todas as operações
    for (int i = 0; i < 3; i++)
    {
        buf[i] |= 0x08; // Liga o backlight
    }

    printf("LCD[%d]: %s\n", row, message);
}