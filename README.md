# Estação de Alerta de Enchente com FreeRTOS

Este projeto implementa uma **estação inteligente de alerta de enchente** utilizando o microcontrolador **RP2040**, o sistema operacional **FreeRTOS** e periféricos embarcados para **detecção simulada de alagamentos**, com exibição visual e sonora.

## Periféricos Utilizados

* **Joystick analógico** – simulação dos sensores de **nível de água** e **chuva**
* **LED RGB (PWM)** – sinalização visual dos modos
* **Buzzer (PWM)** – alarme sonoro em caso de alerta
* **Display OLED SSD1306 (I2C)** – exibição dos valores e status
* **Matriz de LEDs WS2812 5x5 (via PIO)** – ícones gráficos dinâmicos

## Funcionalidades

* Estrutura **multitarefa com FreeRTOS**.
* Modo de alerta visual e sonoro com:
  - LED RGB colorido proporcional ao risco
  - Ícones animados na matriz WS2812
  - Texto claro no display OLED
  - Alarme em caso de emergência
* **Fila unificada (xCommandQueue)** para centralizar os comandos.
* Uso da variável global `lastCommand` para acesso instantâneo pelos periféricos.
* Simulação dos sensores com joystick analógico (ADC0 e ADC1).

## Modos de Operação

1. **Modo Normal**
   * LED verde proporcional ao nível da água
   * Matriz exibe o ícone "O" verde
   * Display mostra "NORMAL"
   * Buzzer desligado

2. **Modo Atenção**
   * LED RGB amarelo fixo
   * Matriz exibe "O" amarelo
   * Display ainda mostra "NORMAL"
   * Buzzer desligado

3. **Modo Alerta**
   * LED RGB vermelho fixo
   * Matriz exibe "X" vermelho
   * Display exibe "**!!! ALERTA !!!**"
   * Buzzer toca a 1000 Hz

## Destaque Técnico

* Sistema construído sobre **FreeRTOS**, com uma arquitetura modular de 6 tasks:
  - `vTaskJoystickRead`: leitura dos ADCs
  - `vTaskLogic`: lógica e thresholds
  - `vTaskRGBLED`: controle PWM RGB
  - `vTaskMatrix`: exibição gráfica PIO/WS2812
  - `vTaskDisplay`: visualização textual OLED
  - `vTaskBuzzer`: alarme sonoro
* Comunicação entre as tarefas usando **uma única fila (`xCommandQueue`)**.
* Periféricos acessam o **último comando com `lastCommand`**, otimizando o desempenho e evitando atrasos por congestionamento da fila.

## Hardware Utilizado

* **RP2040 (Raspberry Pi Pico)**
* Joystick analógico (ou potenciômetros nos pinos ADC0 e ADC1)
* LED RGB comum
* Buzzer passivo
* Matriz WS2812 (5x5 LEDs endereçáveis)
* Display OLED SSD1306 (128x64)

## Software e Ferramentas

* **FreeRTOS** para multitarefa
* **Pico SDK** para acesso ao hardware
* **PIO** para controle da matriz WS2812
* **CMake** e **Ninja** para build
* **VS Code** com extensão Raspberry Pi Pico

## Como Executar o Projeto

1. **Clone** o repositório:

   ```bash
   git clone <url_do_repositorio>
   cd <nome_do_repositorio>
   ```

2. **Ajuste o caminho do SDK e do FreeRTOS** em `CMakeLists.txt`.

3. **Compile o projeto:**

   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

4. **Arraste o arquivo `.uf2`** gerado para sua Raspberry Pi Pico conectada.

5. **Movimente o joystick** para simular os sensores e observar o comportamento da estação.

## Vídeo de Demonstração

🎥 *[link do vídeo no YouTube](https://www.youtube.com/watch?v=mclR3VGTRy0)*

## Licença

Este projeto é **código aberto** e pode ser utilizado livremente para fins educacionais e não comerciais. Créditos ao autor Gabriel Marques.
