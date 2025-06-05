# Acionamento BLDC ATmega328

This repository contains a firmware example for **BLDC** motor control using the **ATmega328P** microcontroller. The code generates a 10 kHz PWM to drive the motor phases based on a potentiometer reading (ADC0) and the rotor position provided by the Hall sensors.

## Build

The project can be built in **Microchip Studio** or directly on the terminal using `avr-gcc`. To generate the `HEX` file run:

```sh
make
```

## Inputs and outputs

| Pin | Function |
|------|--------------------------------------------------------------|
| PC0  | ADC0 - command potentiometer |
| PC1  | Hall 1 (digital input) |
| PC2  | Hall 2 (digital input) |
| PC3  | Hall 3 (digital input) |
| PB0  | AH - high-side MOSFET phase A |
| PB1  | BH - high-side MOSFET phase B |
| PB2  | CH - high-side MOSFET phase C |
| PB3  | AL - low-side MOSFET phase A (AND with PWM) |
| PB4  | BL - low-side MOSFET phase B (AND with PWM) |
| PB5  | CL - low-side MOSFET phase C (AND with PWM) |
| PD5  | OC0B - 10 kHz PWM output |

## Detailed explanation by block

### Global variable `adc_value`
- Stores the result of ADC conversions (0 – 1023).
- Declared as `uint16_t` so it is not limited to 8 bits.
- Updated inside `ISR(ADC_vect)`.

### Pin definitions (`#define`)
- `HALL1`, `HALL2`, `HALL3` (PC1, PC2, PC3): digital inputs connected to the motor Hall sensors.
- `AH_PIN`, `BH_PIN`, `CH_PIN` (PB0, PB1, PB2): high-side MOSFETs, simply switch 5 V on or off.
- `AL_PIN`, `BL_PIN`, `CL_PIN` (PB3, PB4, PB5): low-side MOSFETs, externally ANDed with the PWM generated on PD5.

### PWM trigger and external AND
Timer0 generates a 10 kHz PWM on pin PD5 (OC0B). In hardware the PB3, PB4 and PB5 lines (low-side switches) are externally ANDed with this signal. Therefore the firmware only sets PBx high to enable the desired phase and the PWM on PD5 modulates the current.

### Timer0 initialization (`timer0_pwm_10kHz_init`)
- `DDRD |= (1 << PD5);` sets PD5 (OC0B) as output.
- `TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0B1);`
  - `WGM01=1`, `WGM00=1` → Fast PWM.
  - `COM0B1=1`, `COM0B0=0` → non inverting mode on OC0B.
- `TCCR0B = (1 << WGM02) | (1 << CS01);`
  - `WGM02=1` → completes WGM02:0 = 7 (TOP = OCR0A).
  - `CS01=1` → Timer0 prescaler of 8.
- `OCR0A = 199;` sets TOP = 199 resulting in `f_PWM = 16 MHz/(8·200) = 10 kHz`.
- `OCR0B = 0;` initial duty cycle at zero.

### ADC initialization (`adc_free_running_init`)
- `ADMUX = (1 << REFS0) | …` selects AVcc as reference and channel ADC0.
- `ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);`
  - `ADEN=1` → enables the ADC module.
  - `ADATE=1` → enables auto trigger (Free-Running).
  - `ADIE=1` → enables end-of-conversion interrupt.
  - `ADPS2:0 = 111` → prescaler 128 (≈125 kHz).
- `ADCSRB = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);` → trigger source Free-Running.
- `ADCSRA |= (1 << ADSC);` starts the first conversion.

### Reading the Hall sensors (`hall_state`)
- Reads `PINC` and checks bits `PC1`, `PC2`, `PC3`.
- Builds a 3‑bit value identifying the rotor phase sequence (0 to 7).

### Commutation (`commutate`)
- Copies `adc_value` to `tmp_adc_value` within `cli()`/`sei()` for an atomic read.
- Reads `PORTB` and clears all bits of `AH_PIN`, `BH_PIN`, `CH_PIN`, `AL_PIN`, `BL_PIN`, `CL_PIN`.
- If `tmp_adc_value > 10`, executes `switch(hall)` enabling the proper combination of switches for that phase.
- Otherwise all switches remain off.
- Finally writes `PORTB = tmp_PORTB;`.

### ADC ISR (`ISR(ADC_vect)`)
- Runs at every end of conversion in Free‑Running mode.
- Reads the 10‑bit ADC value.
- Calculates `duty = (adc_value * 199UL) / 1023UL`.
- Updates `OCR0B`, controlling the PWM on PD5.

### Function `main()`
- Sets `PB0`–`PB5` as outputs to drive the MOSFETs.
- Disables interrupts, initializes Timer0 and the ADC, then enables them.
- Enters an endless loop reading `hall_state()` and calling `commutate()`.

### Possible adjustments/use cases
- To reverse the rotation direction, simply invert the assignment sequence inside the `commutate` `switch`.
- To reduce ADC noise, implement a moving average or add an RC filter on PC0.
- To monitor `adc_value` or `duty` over UART, send data in the main loop (keep interrupts only as long as needed for atomic access).
- On Arduino boards disable the default library to avoid conflict with `main()`.

