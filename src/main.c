#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#define max(a,b)
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "uart.h"
#include "ST7735.h"
#include "LCD_GFX.h"

typedef struct ComplexNumber {
    float r;
    float c;
} complex;

volatile int flag = 0;
volatile int read = 0;
volatile unsigned int index = 0;
volatile unsigned int game_type = 0;
volatile unsigned int game_step = 0;
volatile unsigned int count = 0;
volatile unsigned int correct = 0;
volatile unsigned int incorrect = 0;

volatile complex* samples;
volatile complex* output;
volatile float* expected;
volatile float* actual;

const char* D = "D";
const char* Ds = "D#";
const char* C = "C";
const char* Cs = "C#";
const char* A = "A";
const char* B = "B";
const char* Bb = "Bb";
const char* E = "E";
const char* Es = "E#";
const char* F = "F";
const char* Fs = "F#";
const char* G = "G";
const char* Gs = "G#";
const char* Q = "?";

#define SAMPLE_SIZE 256
#define BAUD_RATE 9600
#define BAUD_PRESCALER ( (F_CPU / (BAUD_RATE*16UL)) - 1 )

void dft(volatile complex *sample_list, volatile complex *out, int high) {
    if (high <= 1) {
        return;
    } else {
        complex *even = out;
        complex *odd = out + (high / 2);
        for (int i = 0; i < high / 2; i++) {
            even[i] = sample_list[2 * i];
            odd[i] = sample_list[2 * i + 1];
        }
        dft(even, sample_list, high / 2);
        dft(odd, sample_list, high / 2);
        double pr;
        double pc;
        double qr;
        double qc;
        double rf;
        double cf;
        double qor;
        double qoc;
        for (int i = 0; i < high / 2; i++) {
            pr = even[i].r;
            pc = even[i].c;
            qr = odd[i].r;
            qc = odd[i].c;
            rf = cos((2 * M_PI * i) / ((double)high));
            cf = -sin((2 * M_PI * i) / ((double)high));
            qor = (rf * qr) - (cf * qc);
            qoc = (rf * qc) + (cf * qr);
            sample_list[i] = (complex){pr + qor, pc + qoc};
            sample_list[i + (high / 2)] = (complex){pr - qor, pc - qoc};
//            char buf[256];
//            sprintf(buf, "Sample: %f\n", sample_list[i].r);
//            sprintf(buf, "Sample: %f\n", sample_list[i + high/2].r);
//            UART_STRING(buf);
        }
    }
}

const char* get_note(float freq) {
    if (freq > 134 && freq < 145) {
        return D;
    } else if (freq < 155 && freq > 144) {
        return Ds;
    } else if (freq < 165 && freq > 154) {
        return E;
    } else if (freq < 180 && freq > 164) {
        return F;
    } else if (freq < 200 && freq > 190) {
        return G;
    } else if (freq < 225 && freq > 210) {
        return A;
    } else {
        return Q;
    }
}

void clock_init(void) {
    cli();

    //OC0A/OC0B disconnected
    TCCR0A &= ~(1<<COM0A1);
    TCCR0A &= ~(1<<COM0A0);
    TCCR0A &= ~(1<<COM0B1);
    TCCR0A &= ~(1<<COM0B0);
    //set clock to Normal Mode
    TCCR0A &= ~(1<<WGM00);
    TCCR0A &= ~(1<<WGM01);
    TCCR0B &= ~(1<<WGM02);
    // turn clock on without prescaling
    TCCR0B &= ~(1<<CS02);
    TCCR0B |= (1<<CS00);
    TCCR0B &= ~(1<<CS01);

    //activate overflow interrupt
    TIMSK0 |= (1<<TOIE0);
    sei();
}

void switch_init(void) {
    cli();
    EICRA |= (1<<ISC01);
    EICRA |= (1<<ISC00);
    EIMSK |= (1<<INT0);
    sei();
}

void button_init(void) {

}

void adc_init(void) {
    //setup ADC
    cli();
    PRR0 &= ~(1<<PRADC);

    //select Vref
    ADMUX |= (1<<REFS0);
    ADMUX &= ~(1<<REFS1);

    //set ADC Clock div by 128
    ADCSRA |= (1<<ADPS0);
    ADCSRA |= (1<<ADPS1);
    ADCSRA |= (1<<ADPS2);

    //select channel 0 or 1 based on D4 read
    ADMUX &= ~(1<<MUX0);
    ADMUX &= ~(1<<MUX1);
    ADMUX &= ~(1<<MUX2);
    ADMUX &= ~(1<<MUX3);

    //Set to free running
    ADCSRA |= (1<<ADATE);
    ADCSRB &= ~(1<<ADTS0);
    ADCSRB &= ~(1<<ADTS1);
    ADCSRB &= ~(1<<ADTS2);

    //disable input buffer on ADC pin
    DIDR0 |= (1<<ADC0D);

    //enable ADC
    ADCSRA |= (1<<ADEN);
    ADCSRA |= (1<<ADIE);

    ADCSRA |= (1<<ADSC);
    sei();
}

void timer1_init(void) {
    cli();
    //set I/O pins
    //DDRD |= (1<<DDD2);
    //PORTD &= ~(1<<PORTD2);


    //turn off output compare port operation
    TCCR1A &= ~(1<<COM1A1);
    TCCR1A &= ~(1<<COM1A0);
    TCCR1A &= ~(1<<COM1B1);
    TCCR1A &= ~(1<<COM1B0);
    TCCR1A &= ~(1<<COM1C1);
    TCCR1A &= ~(1<<COM1C0);

    //waveform generation is normal
    TCCR1B &= ~(1<<WGM13);
    TCCR1B &= ~(1<<WGM12);
    TCCR1A &= ~(1<<WGM11);
    TCCR1A &= ~(1<<WGM10);

    //pre-scaling set to 1024
    TCCR1B |= (1<<CS12);
    TCCR1B &= ~(1<<CS11);
    TCCR1B &= ~(1<<CS10);

    //TCCR1B |= (1<<ICES1);
    //TCCR1B |= (1<<ICNC1);
    //TIMSK1 |= (1<<ICIE1);
    //activate overflow interrupt
    TIMSK1 |= (1<<TOIE1);

    sei();
}

void init_sample(void) {
    samples = (complex*) malloc(sizeof(complex)*SAMPLE_SIZE);
    output = (complex*) malloc(sizeof(complex)*SAMPLE_SIZE);
    expected = (float*) malloc(sizeof(float)*19);
    actual = (float*) malloc(sizeof(float)*19);
    float comeasyouare[19] = {0, 0, 0, 140, 150, 160,
                              196, 160, 196, 160, 160,
                              150, 140, 220, 140, 140, 220,
                              140, 150};
    for (int i = 3; i < 19; i++) { expected[i] = comeasyouare[i]; }
    for (int i = 3; i < 19; i++) { actual[i] = -1; }

}

void sample_freq(void) {
    if (index == SAMPLE_SIZE && game_type == 1) {
        cli();
        flag = -1;
        index = 0;
        char buf[256];
        dft(samples, output, SAMPLE_SIZE);
//    for (int i = 0; i < SAMPLE_SIZE; i++) {
//        sprintf(buf, "Sample: %6.3f\t", samples[i].r);
//        UART_STRING(buf);
//    }
        float max = -1;
        int imax = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            float curr = samples[i].r;
            if (curr - max > 1 && i != 0) {
                max = curr;
                imax = i;
            }
        }
        // printf("imax: %d\n", imax);
        sprintf(buf, "Approximate frequency: %f\n", (2400.0 / SAMPLE_SIZE) * imax);
        UART_STRING(buf);
        flag = 0;
        sei();
    } else if (index == SAMPLE_SIZE && game_type == 2 && game_step < 19) {
        cli();
        flag = -1;
        index = 0;
        if (actual[game_step] - 1 > -1) {
            flag = 0;
            return;
        }
        dft(samples, output, SAMPLE_SIZE);
//    for (int i = 0; i < SAMPLE_SIZE; i++) {
//        sprintf(buf, "Sample: %6.3f\t", samples[i].r);
//        UART_STRING(buf);
//    }
        float max = -1;
        int imax = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            float curr = samples[i].r;
            if (curr - max > 1 && i != 0) {
                max = curr;
                imax = i;
            }
        }
        // printf("imax: %d\n", imax);
        actual[game_step] = (2400.0 / SAMPLE_SIZE) * imax;
        flag = 0;
        sei();
    }
    index++;
    float out = ADC;
    if (out < 0) out = 0;
    samples[index].r = out;
    samples[index].c = 0;
}

void externint_init(void) {
    cli();
    EIMSK |= (1<<0);
    EICRA |= (1<<0);
    EICRA |= (1<<1);
    EIFR |= (1<<0);
    sei();
}

ISR(ADC_vect) {
    if (flag == 1 && (read++) == 3) {
        sample_freq();
        read = 0;
    }
    ADCSRA |= (1<<ADIF);
}

//ISR(INT0_vect) {
//    UART_SEND('x');
//    PORTD ^= (1<<PORTD3);
//}

ISR(TIMER1_OVF_vect) {
//    char buf[256];
//    if ( fabs((9600.0 / SAMPLE_SIZE) * imax - expected[game_step]) <  38) {
//        sprintf(buf, "Correct: %f\n", (9600.0 / SAMPLE_SIZE) * imax);
//        UART_STRING(buf);
//        correct++;
//    } else {
//        sprintf(buf, "Wrong; Got: %f Expected: %f \n", (9600.0 / SAMPLE_SIZE) * imax, expected[game_step]);
//        UART_STRING(buf);
//        incorrect++;
//    }
    if (game_step > 2) {
        int offset = game_step - 3;
        int up = (1 + (offset/8));
        if (fabs(actual[game_step] - expected[game_step]) <  11) {
            LCD_drawBlock(0 + (20*(offset%8)),(64 * (up -1)), 20 + (20*(offset%8)), 64 * up, GREEN);
            correct++;
        } else {
            LCD_drawBlock(0 + (20*(offset%8)),(64 * (up -1)), 20 + (20*(offset%8)), 64 * up, RED);
            incorrect++;
        }
    }

    PORTD |= (1<<PORTD1);
    Delay_ms(10);
    PORTD &= ~(1<<PORTD1);

    game_step++;
}



int main(void) {
    lcd_init();
    LCD_setScreen(BLUE);
    LCD_drawString(20, 64, "Welcome to Bass Villain!", YELLOW, BLACK);
    DDRD &= ~(1<<DDD3);
    PORTD &= ~(1<<PORTD3);
    DDRD &= ~(1<<DDD0);
    PORTD &= ~(1<<PORTD0);
    while(!game_type) {
        if (PIND & (1<<PIND0)) {
            game_type = 1;
        } else if (PIND & (1<<PIND3)) {
            game_type = 2;
        }
    };
    init_sample();
    adc_init();
    UART_INIT(BAUD_PRESCALER);
    DDRD &= ~(1<<DDD2);
    PORTD &= ~(1<<PORTD2);
    DDRD |= (1<<DDD1);
    PORTD &= ~(1<<PORTD1);
    if (game_type == 1) {
        LCD_setScreen(CYAN);
        LCD_drawString(50, 64, "Free Play Mode!", WHITE, CYAN);
        while(1) {
            if (PIND & (1<<PIND2) && flag == 0) { flag = 1;}
        }
    } else if (game_type == 2) {
        timer1_init();
        cli();
        LCD_setScreen(BLACK);
        for (int i = 3; i < 19; i++) {
            int offset = (i - 3);
            int up = (1 + (offset/8));
            LCD_drawString(10 + 20*(offset % 8), 32 + 64 * (up -1), get_note(expected[i]), YELLOW, BLACK);
        }
        sei();
        while(1) {
            if (PIND & (1<<PIND2) && flag == 0) { flag = 1;}
            if (game_step > 18) {
                cli();
                char buf[256];
                for (int i = 3; i < 19; i++) {
                    sprintf(buf, "Expected: %f Actual: %f \n", expected[i], actual[i]);
                    UART_STRING(buf);
                }
                sprintf(buf, "Correct: %d Incorrect: %d \n", correct, incorrect);
                LCD_drawString(20, 64, buf, WHITE, BLACK);
                sei();
                break;
            }
        }
    }
}