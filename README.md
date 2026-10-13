# Trabalho PID

### Contribuidores
- Gustavo Pivoto Ambrósio
- Marcelo Fróes Padilha Filho
- Miguel Vianna Streva

### Professor
- Samuel Baraldi Mafra

### Monitor
- Lucas Henrique Ribeiro

## Parte teórica do trabalho

### 1. Funcionamento da planta

A planta consiste em um motor de corrente contínua (CC), cujo objetivo é controlar sua velocidade de rotação utilizando um controlador PID.

O controle é realizado através de um sinal PWM (Pulse Width Modulation), que permite variar a tensão média aplicada ao motor e, consequentemente, sua velocidade. O controlador PID compara a velocidade atual do motor com o valor desejado (SetPoint) e ajusta o PWM para reduzir o erro.

Em uma implementação real, poderiam ser utilizados os seguintes componentes:
- Sensor: encoder incremental, responsável por medir a velocidade de rotação do motor em RPM;
- Atuador: motor CC acionado por um driver de potência, que controla a energia fornecida ao motor;
- Controlador: microcontrolador, como STM32 ou ESP32, responsável por executar o algoritmo PID, ler o sensor e gerar o sinal PWM.

Dessa forma, o sistema funciona em malha fechada, ajustando continuamente a velocidade do motor para mantê-la próxima do SetPoint definido.

### 2. Variáveis do processo e faixas de operação

As variáveis envolvidas no controle de velocidade do motor CC são:
- Variável controlada: velocidade de rotação do motor, medida em RPM (é a grandeza que desejamos regular);
- Variável manipulada: duty cycle do sinal PWM, responsável por alterar a tensão média aplicada ao motor;
- SetPoint: velocidade desejada para o motor, definida pelo usuário;
- Perturbações: variações de carga mecânica no eixo, alterações na tensão de alimentação, atrito e mudanças de temperatura.

As faixas de operação são apresentadas na tabela abaixo:

| Grandeza | Faixa de operação |
| --- | --- |
| Velocidade do motor | 0 até a velocidade máxima nominal do motor |
| PWM | 0% a 100% |
| SetPoint | 0 até a velocidade máxima operacional |

## Parte prática do trabalho

### 7. Interface

Foi desenvolvida uma interface em Python para facilitar a análise do sistema. A interface permite:

- selecionar o método de sintonia do controlador;
- ajustar manualmente os parâmetros do PID;
- visualizar a resposta do sistema em malha fechada;
- comparar os métodos CHR e ITAE;
- carregar o dataset experimental para análise da planta;
- exportar o gráfico gerado.

### Como executar

1. Acesse a pasta `interface`.

2. Dentro dessa pasta, execute:
```bash
python -m venv .venv
```

3. Depois, ative o ambiente:
```bash
source .venv/bin/activate
```

4. Com o ambiente virtual ativado:
```bash
python -m pip install -r requirements.txt
```

5. Por fim, execute a interface:
```bash
python interface.py
```
