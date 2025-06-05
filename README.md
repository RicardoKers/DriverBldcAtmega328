# Teste1

Este repositório contém um exemplo de firmware para controle de motor **BLDC** utilizando o microcontrolador **ATmega328P**. O código gera PWM em 10 kHz para acionar as fases do motor com base na leitura de um potenciômetro (ADC0) e na posição do rotor informada pelos sensores Hall.

## Compilação

O projeto pode ser compilado no **Microchip Studio** ou diretamente no terminal com `avr-gcc`. Para gerar o arquivo `HEX` execute:

```sh
make
```

O arquivo `wiring.txt` descreve o esquema de conexão recomendado.

## Explicações detalhadas por bloco

### Variável global `adc_value`
- Armazena o resultado das conversões ADC (0 – 1023).
- Declarada como `uint16_t` para não limitar a 8 bits.
- Atualizada na `ISR(ADC_vect)`.

### Definições de pinos (`#define`)
- `HALL1`, `HALL2`, `HALL3` (PC1, PC2, PC3): entradas digitais conectadas aos sensores Hall do motor.
- `AH_PIN`, `BH_PIN`, `CH_PIN` (PB0, PB1, PB2): chaves superiores (high-side MOSFETs), apenas ligam 5 V ou desligam.
- `AL_PIN`, `BL_PIN`, `CL_PIN` (PB3, PB4, PB5): chaves inferiores (low-side MOSFETs), ANDadas externamente com o PWM gerado em PD5.

### Inicialização do Timer0 (`timer0_pwm_10kHz_init`)
- `DDRD |= (1 << PD5);` configura PD5 (OC0B) como saída.
- `TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0B1);`
  - `WGM01=1`, `WGM00=1` → Fast PWM.
  - `COM0B1=1`, `COM0B0=0` → modo não inversor em OC0B.
- `TCCR0B = (1 << WGM02) | (1 << CS01);`
  - `WGM02=1` → completa WGM02:0 = 7 (TOP = OCR0A).
  - `CS01=1` → prescaler do Timer0 igual a 8.
- `OCR0A = 199;` define TOP = 199 resultando em `f_PWM = 16 MHz/(8·200) = 10 kHz`.
- `OCR0B = 0;` duty cycle inicial em zero.

### Inicialização do ADC (`adc_free_running_init`)
- `ADMUX = (1 << REFS0) | …` seleciona AVcc como referência e o canal ADC0.
- `ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);`
  - `ADEN=1` → habilita o módulo ADC.
  - `ADATE=1` → habilita auto-trigger (Free-Running).
  - `ADIE=1` → habilita interrupção de fim de conversão.
  - `ADPS2:0 = 111` → prescaler 128 (≈125 kHz).
- `ADCSRB = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);` → fonte de gatilho Free-Running.
- `ADCSRA |= (1 << ADSC);` dispara a primeira conversão.

### Leitura dos sensores Hall (`hall_state`)
- Lê `PINC` e verifica bits `PC1`, `PC2`, `PC3`.
- Monta um valor de 3 bits que identifica a sequência de fases do rotor (0 a 7).

### Comutação de fases (`commutate`)
- Copia `adc_value` para `tmp_adc_value` dentro de `cli()`/`sei()` garantindo leitura atômica.
- Lê `PORTB` e limpa todos os bits de `AH_PIN`, `BH_PIN`, `CH_PIN`, `AL_PIN`, `BL_PIN`, `CL_PIN`.
- Se `tmp_adc_value > 10`, executa `switch(hall)` habilitando a combinação de chaves para a fase correspondente.
- Caso contrário, mantém todas as chaves desligadas.
- Por fim, grava `PORTB = tmp_PORTB;`.

### ISR do ADC (`ISR(ADC_vect)`)
- Executa a cada fim de conversão em Free-Running.
- Lê o valor de 10 bits do ADC.
- Calcula `duty = (adc_value * 199UL) / 1023UL`.
- Atualiza `OCR0B`, controlando o PWM em PD5.

### Função `main()`
- Configura `PB0`–`PB5` como saídas para controlar os MOSFETs.
- Desabilita interrupções, inicializa Timer0 e ADC e, em seguida, habilita-as.
- Entra em laço infinito onde lê `hall_state()` e chama `commutate()`.

### Possíveis ajustes/casos de uso
- Para inverter o sentido de rotação, basta inverter a sequência de atribuições dentro do `switch` de `commutate`.
- Para suavizar ruídos do ADC, implemente média móvel ou adicione filtro RC em PC0.
- Para monitorar `adc_value` ou `duty` via UART, envie dados no loop principal (utilize interrupções apenas o tempo necessário para acessar variáveis atômicas).
- Em placas Arduino, desabilite a biblioteca padrão para não conflitar com `main()`.

