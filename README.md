# Sistema de Monitoramento Ambiental para Agricultura de Precisão

## Visão Geral do Projeto
Este projeto consiste em um sistema embarcado de monitoramento ambiental voltado para agricultura de precisão. O dispositivo utilizará sensores para coletar dados como temperatura, umidade do solo e luminosidade, auxiliando agricultores na tomada de decisões baseadas em dados precisos.

## Componentes Principais

### Sensores:
- Sensor de temperatura e umidade do ar (DHT11/DHT22)
- Sensor de umidade do solo
- Sensor de luminosidade (LDR)

### Processamento:
- Microcontrolador da placa BitDogLab
- Armazenamento temporário de dados em memória

### Saída/Comunicação:
- Display LCD para visualização em tempo real
- LEDs indicadores para alertas
- Possibilidade de comunicação serial para download dos dados

## Funcionalidades

### Coleta de Dados:
- Leitura periódica de temperatura ambiente
- Medição de umidade do solo
- Monitoramento do nível de luminosidade

### Processamento:
- Cálculo de médias e tendências
- Comparação com valores ideais pré-configurados

### Alertas e Visualização:
- Indicação visual (LEDs) para condições fora do ideal
- Exibição de leituras atuais no display LCD
- Histórico de medições (últimas 24h)