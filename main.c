/**
 * Controle de Motor BLDC em ATmega328P @ 16 MHz
 * 
 * - Leitura do ADC0 (PC0) em modo Free-Running, converte 0 – 5 V em 0 – 100 % de PWM
 *   (0 – 199 no OCR0B), gerando PWM em 10 kHz no pino OC0B (PD5).
 * - Sensores Hall em PC1, PC2 e PC3 definem posição do rotor (3 bits).
 * - Chaves superiores (High-Side) em PB0, PB1, PB2 — acionamento digital on/off.
 * - Chaves inferiores (Low-Side) em PB3, PB4, PB5 — gating via AND externo com PWM em PD5.
 * - Toda atualização de duty cycle do PWM ocorre na ISR do ADC; a rotina de comutação
 *   roda no loop principal, lendo sensores Hall e habilitando as chaves conforme posição
 *   do rotor, desde que adc_value > threshold (≈10).
 *
 * Deve-se conectar externamente cada linha PB3/4/5 a uma porta AND, junto com PD5 (PWM OC0B),
 * para que só haja acionamento do MOSFET se ambas as entradas do AND estiverem em nível alto.
 *
 * Para compilar no Microchip Studio (Atmel Studio):
 * - Criar projeto “AVR C Executable” para ATmega328P.
 * - Definir F_CPU como 16000000UL (no código ou nas Symbols do projeto).
 * - Gravar fuses para cristal externo de 16 MHz (se aplicável).
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/* 
 * ----------------------------------------------------------------------------
 * Variável global para armazenar o valor do ADC (10 bits, 0–1023)
 * Atualizada na ISR(ADC_vect). Na função de comutação, lê-se com cli()/sei() para 
 * garantir leitura atômica de 16 bits.
 * ----------------------------------------------------------------------------
 */
uint16_t adc_value;

/*
 * ----------------------------------------------------------------------------
 * Definições de pinos (usando nomenclatura de registradores AVR):
 * ----------------------------------------------------------------------------
 *   - Sensores Hall: PC1, PC2, PC3 (inputs digitais — MUX1:0 do ADC não está usando
 *     estes pinos, pois ADC0 = PC0)
 *   - High-Side MOSFETs: PB0 (Aʰ), PB1 (Bʰ), PB2 (Cʰ) → acionamento digital on/off
 *   - Low-Side MOSFETs: PB3 (Aˡ), PB4 (Bˡ), PB5 (Cˡ) → gating por AND externo com PWM (PD5)
 */
#define HALL1   PC1
#define HALL2   PC2
#define HALL3   PC3

#define AH_PIN  PB0
#define BH_PIN  PB1
#define CH_PIN  PB2

#define AL_PIN  PB3
#define BL_PIN  PB4
#define CL_PIN  PB5

/*
 * ----------------------------------------------------------------------------
 * Inicialização do Timer0 para geração de PWM em 10 kHz no canal OC0B (PD5)
 * ----------------------------------------------------------------------------
 *
 * Configurações:
 *  - Modo Fast PWM, TOP = OCR0A → WGM02:0 = 1 1 1 (modo 7)
 *  - Saída não inversora em OC0B → COM0B1:0 = 1 0
 *  - Prescaler = 8 → CS02:0 = 0 1 0
 *  - OCR0A = 199 → define TOP = 199 → f_PWM = F_CPU / (N·(TOP+1))
 *                       = 16 MHz / (8·200) = 10 000 Hz
 *  - OCR0B ajusta duty cycle entre 0 e 199 (0 – 100 %)
 *  - PD5 deve ser configurado como saída
 */
static void timer0_pwm_10kHz_init(void)
{
    // Configurar PD5 (OC0B) como saída
    DDRD |= (1 << PD5);

    // TCCR0A: WGM01=1, WGM00=1 → Fast PWM; COM0B1=1, COM0B0=0 → non-inverting no OC0B
    TCCR0A = (1 << WGM01) | (1 << WGM00)
           | (1 << COM0B1);

    // TCCR0B: WGM02=1 → completa WGM02:0 = 7; CS01=1 → prescaler = 8
    TCCR0B = (1 << WGM02)
           | (1 << CS01);

    // Define TOP em OCR0A para gerar exatamente 10 kHz
    OCR0A = 199;
    // Inicializa duty cycle em 0 (saída sempre LOW até primeira ISR do ADC)
    OCR0B = 0;
}

/*
 * ----------------------------------------------------------------------------
 * Inicialização do ADC em modo Free-Running (canal ADC0, PC0) com interrupção
 * ----------------------------------------------------------------------------
 *
 * Configurações:
 *  - REFS0=1, REFS1=0 → referência AVcc (5 V)
 *  - MUX3:0 = 0000 → canal ADC0 (pino PC0)
 *  - ADLAR = 0 → resultado justificado à direita (10 bits em ADC[9:0])
 *  - ADPS2:0 = 1 1 1 → prescaler = 128 → F_ADC ≈ 16 MHz / 128 ≈ 125 kHz
 *  - ADATE = 1 → auto-trigger (Free-Running Mode)
 *  - ADIE = 1 → habilita interrupção no fim da conversão (ADC_vect)
 *  - ADTS2:0 = 0 0 0 → fonte de trigger = Free-Running
 *  - ADSC = 1 → inicia a primeira conversão; a partir daí, cada fim gera outra
 */
static void adc_free_running_init(void)
{
    // Selecionar referência AVcc e canal ADC0 (PC0); ADLAR=0 → just. direita
    ADMUX = (1 << REFS0)
          | (0 << REFS1)
          | (0 << ADLAR)
          | (0 << MUX3)
          | (0 << MUX2)
          | (0 << MUX1)
          | (0 << MUX0);

    // Habilita ADC, Auto Trigger, Interrupt on Conversion, prescaler = 128
    ADCSRA = (1 << ADEN)
           | (1 << ADATE)
           | (1 << ADIE)
           | (1 << ADPS2)
           | (1 << ADPS1)
           | (1 << ADPS0);

    // Modo de gatilho = Free-Running (ADTS2:0 = 0 0 0)
    ADCSRB = (0 << ADTS2)
           | (0 << ADTS1)
           | (0 << ADTS0);

    // Inicia a primeira conversão; após isso, o ADC fica em free-running
    ADCSRA |= (1 << ADSC);
}

/*
 * ----------------------------------------------------------------------------
 * Lê o estado dos sensores Hall (3 bits) e retorna valor de 0 a 7:
 *   bit2 = HALL1 (PC1), bit1 = HALL2 (PC2), bit0 = HALL3 (PC3)
 * ----------------------------------------------------------------------------
 * Retorno:
 *   0b000 a 0b111 conforme sinais digitais dos pinos PC1, PC2, PC3
 */
static inline uint8_t hall_state(void)
{
    uint8_t state = 0;
    if (PINC & (1 << HALL1)) state |= 0b100;
    if (PINC & (1 << HALL2)) state |= 0b010;
    if (PINC & (1 << HALL3)) state |= 0b001;
    return state;
}

/*
 * ----------------------------------------------------------------------------
 * Função de comutação baseado em leitura de Hall (rotor em 6 passos)
 * ----------------------------------------------------------------------------
 *
 * 1) Copia 'adc_value' de maneira atômica (cli()/sei()) para 'tmp_adc_value'.
 * 2) Se tmp_adc_value > 10 (≈ 0,05 V), habilita as chaves High-Side e Low-Side
 *    correspondentes à fase, conforme código Gray dos sensores Hall.
 * 3) Se tmp_adc_value ≤ 10, mantém todas as saídas em 0 (desliga todas as fases).
 * 4) Em hardware, cada linha de Low-Side (PB3, PB4, PB5) deve passar por AND
 *    com o sinal PWM em PD5 para modular a tensão do motor.
 *
 * Sequência de comutação padrão (assumindo 120° elétricos entre cada sensor Hall):
 *   Hall = 0b001 (1): Cʰ, Bˡ
 *   Hall = 0b101 (5): Aʰ, Bˡ
 *   Hall = 0b100 (4): Aʰ, Cˡ
 *   Hall = 0b110 (6): Bʰ, Cˡ
 *   Hall = 0b010 (2): Bʰ, Aˡ
 *   Hall = 0b011 (3): Cʰ, Aˡ
 *
 * Ajuste a ordem caso o sentido de rotação fique invertido.
 */
static void commutate(uint8_t hall)
{
    uint16_t tmp_adc_value;

    //-Cópia atômica de 16 bits de 'adc_value'-----------------------------------
    cli();                  // desabilita interrupções
    tmp_adc_value = adc_value;
    sei();                  // habilita interrupções

    //-Lê PORTB atual para evitar sobrescrever pinos não utilizados---------------
    uint8_t tmp_PORTB = PORTB;

    // Limpa todos os bits de High-Side e Low-Side (0 → desligado)
    tmp_PORTB &= ~((1 << AH_PIN) | (1 << BH_PIN) | (1 << CH_PIN)
                 | (1 << AL_PIN) | (1 << BL_PIN) | (1 << CL_PIN));

    if (tmp_adc_value > 10)
    {
        // Se acima do threshold, comuta conforme valor de 'hall'
        switch (hall)
        {
            case 0b001:  // Hall = 1 → Cˡ e Bʰ
                tmp_PORTB |= (1 << CH_PIN);  // High-Side C = 1
                tmp_PORTB |= (1 << BL_PIN);  // Low-Side B = 1 (gate AND receberá PWM)
                break;

            case 0b101:  // Hall = 5 → Bˡ e Aʰ
                tmp_PORTB |= (1 << AH_PIN);  // High-Side A = 1
                tmp_PORTB |= (1 << BL_PIN);  // Low-Side B = 1
                break;

            case 0b100:  // Hall = 4 → Cˡ e Aʰ
                tmp_PORTB |= (1 << AH_PIN);  // High-Side A = 1
                tmp_PORTB |= (1 << CL_PIN);  // Low-Side C = 1
                break;

            case 0b110:  // Hall = 6 → Cˡ e Bʰ
                tmp_PORTB |= (1 << BH_PIN);  // High-Side B = 1
                tmp_PORTB |= (1 << CL_PIN);  // Low-Side C = 1
                break;

            case 0b010:  // Hall = 2 → Aˡ e Bʰ
                tmp_PORTB |= (1 << BH_PIN);  // High-Side B = 1
                tmp_PORTB |= (1 << AL_PIN);  // Low-Side A = 1
                break;

            case 0b011:  // Hall = 3 → Aˡ e Cʰ
                tmp_PORTB |= (1 << CH_PIN);  // High-Side C = 1
                tmp_PORTB |= (1 << AL_PIN);  // Low-Side A = 1
                break;

            default:
                // Se leitura inválida (0, 7 ou outros), não comuta nenhuma fase
                break;
        }
    }
    // Se tmp_adc_value <= 10, mantém todas as chaves desligadas (tmp_PORTB já limpo)

    // Atualiza PORTB com a nova configuração
    PORTB = tmp_PORTB;
}

/*
 * ----------------------------------------------------------------------------
 * ISR do ADC: atualiza o duty cycle do PWM (OCR0B) proporcional ao valor do ADC
 * ----------------------------------------------------------------------------
 *
 * Executa a cada fim de conversão (modo Free-Running):
 * 1) Lê valor ADC de 10 bits (0–1023).
 * 2) Calcula duty = floor(adc_value * 199 / 1023) → mapeia 0 – 1023 em 0 – 199.
 * 3) Atualiza OCR0B (valor 8 bits) para definir duty cycle no Timer0.
 */
ISR(ADC_vect)
{
    uint16_t duty;

    // Lê resultado de 10 bits do ADC (macro ADC faz leitura de ADCL e ADCH)
    adc_value = ADC;

    // Cálculo de duty (0–199) para 0–100 % de 10 kHz
    duty = (uint16_t)((adc_value * 199UL) / 1023UL);

    // Atualiza OCR0B (8 bits). Casting implícito descarta os bits superiores.
    OCR0B = (uint8_t)duty;
}

int main(void)
{
    //----------------------------------------------------------------------------- 
    // Configuração de pinos:
    //  - Configura PB0–PB5 como saída (High-Side e Low-Side MOSFETs)
    //  - PD5 será configurado em timer0_pwm_10kHz_init()
    //  - PC1–PC3 já são entradas por padrão (sensores Hall)
    //-----------------------------------------------------------------------------
    DDRB |= (1 << AH_PIN) | (1 << BH_PIN) | (1 << CH_PIN)
          | (1 << AL_PIN) | (1 << BL_PIN) | (1 << CL_PIN);

    //----------------------------------------------------------------------------- 
    // Desabilita interrupções globais durante configuração de periféricos
    //-----------------------------------------------------------------------------
    cli();

    //-----------------------------------------------------------------------------
    // Inicializa Timer0 para gerar PWM em 10 kHz no pino PD5 (OC0B)
    //-----------------------------------------------------------------------------
    timer0_pwm_10kHz_init();

    //-----------------------------------------------------------------------------
    // Inicializa ADC em modo Free-Running no canal ADC0 (PC0) com interrupção
    //-----------------------------------------------------------------------------
    adc_free_running_init();

    //-----------------------------------------------------------------------------
    // Habilita interrupções globais para permitir ISR do ADC
    //-----------------------------------------------------------------------------
    sei();

    //----------------------------------------------------------------------------- 
    // Loop principal vazio: 
    //   - Apenas lê sensores Hall e chama commutate() continuamente.
    //   - Toda lógica de modulação de tensão (duty) está na ISR do ADC.
    //-----------------------------------------------------------------------------
    while (1)
    {
        uint8_t hall = hall_state();
        commutate(hall);
        // Pode-se inserir aqui um pequeno _NOP_ ou outra lógica não crítica
    }

    // Nunca alcançado
    return 0;
}
