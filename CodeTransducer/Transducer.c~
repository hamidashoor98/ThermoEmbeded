/*******************************************************
CodeWizardAVR V3.12 Advanced
Project : 
Version : 
Date    : 9/8/2020
Author  : M.Mashaghi 
Company : MegaMouj

Chip type               : ATmega48A
AVR Core Clock frequency: 4.000000 MHz
*******************************************************/
#include <mega48a.h>
#include <delay.h>

#define DataDirection PORTC.1

#define _7seg1G PORTD.5

#define _7seg2A PORTB.0
#define _7seg2B PORTB.1
#define _7seg2C PORTD.2
#define _7seg2D PORTD.4
#define _7seg2E PORTD.3
#define _7seg2F PORTD.7
#define _7seg2G PORTD.6
#define _7seg2dp PORTC.5

#define _7seg3A PORTB.4
#define _7seg3B PORTB.5
#define _7seg3C PORTC.2
#define _7seg3D PORTC.3
#define _7seg3E PORTC.4
#define _7seg3F PORTB.3
#define _7seg3G PORTB.2

void ShowValue(signed char val);
void ShowFloatValue(float val);
void Report(void);

unsigned char Address;
eeprom unsigned char AddressSaved;
unsigned char rom_codes[1][9];
float T1, T2;
unsigned char status;


union
{float f;
 unsigned long int ui32;
 unsigned int ui16[2];
 unsigned char ui8[4];
}format_converter_32;

// 1 Wire Bus interface functions
#include <1wire.h>

// DS1820 Temperature Sensor functions
#include <ds1820.h>

// Declare your global variables here

#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

// USART Receiver buffer
#define RX_BUFFER_SIZE0 256
char rx_buffer0[RX_BUFFER_SIZE0];

#if RX_BUFFER_SIZE0 <= 256
unsigned char rx_wr_index0=0,rx_rd_index0=0;
#else
unsigned int rx_wr_index0=0,rx_rd_index0=0;
#endif

#if RX_BUFFER_SIZE0 < 256
unsigned char rx_counter0=0;
#else
unsigned int rx_counter0=0;
#endif

// This flag is set on USART Receiver buffer overflow
bit rx_buffer_overflow0;

// USART Receiver interrupt service routine
interrupt [USART_RXC] void usart_rx_isr(void)
{
char status,data;
status=UCSR0A;
data=UDR0;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
   rx_buffer0[rx_wr_index0++]=data;
#if RX_BUFFER_SIZE0 == 256
   // special case for receiver buffer size=256
   if (++rx_counter0 == 0) rx_buffer_overflow0=1;
#else
   if (rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
   if (++rx_counter0 == RX_BUFFER_SIZE0)
      {
      rx_counter0=0;
      rx_buffer_overflow0=1;
      }
#endif
   }
}

#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void)
{
char data;
while (rx_counter0==0);
data=rx_buffer0[rx_rd_index0++];
#if RX_BUFFER_SIZE0 != 256
if (rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
#endif
#asm("cli")
--rx_counter0;
#asm("sei")
return data;
}
#pragma used-
#endif

// Standard Input/Output functions
#include <stdio.h>

// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (1<<ADLAR))

// Read the 8 most significant bits of the AD conversion result
unsigned char read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
delay_us(10);
ADCSRA|=(1<<ADSC);
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCH;
}

void main(void)
{
signed char i, devices;
float f;
unsigned char analog;

// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
DDRB=0x3F;PORTB=0x3F;
DDRC=0x3E;PORTC=0x3C;
DDRD=0xFC;PORTD=0xFC;

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART0 Mode: Asynchronous
// USART Baud Rate: 9600
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x19;

// ADC initialization
// ADC Clock frequency: 1000.000 kHz
// ADC Voltage Reference: AVCC pin
// Only the 8 most significant bits of the AD conversion result are used
DIDR0=0;
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);
ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

// 1 Wire Bus initialization
// 1 Wire Data port: PORTC
// 1 Wire Data bit: 0
// Note: 1 Wire port settings are specified in the
// Project|Configure|C Compiler|Libraries|1 Wire menu.
w1_init();

// Global enable interrupts
#asm("sei")

//RetreiveAddress();

devices=w1_search(0xf0,rom_codes);
if (devices) ShowValue(15);
else ShowValue(-3);

delay_ms(2000);
//if (ds1820_init(0,-125,125,DS18B20_10BIT_RES)) ShowValue(15);
//else ShowValue(-1);

//while(1) {
//ShowValue(read_adc(6)/3);
//putchar(read_adc(6));
//delay_ms(50);
//}

while (1)
    {
        T1 = ds1820_temperature_10(0);
        //T1 /= 100;
        T1 /= 75; //this is so strange!!! (75 is the result of caliberation with boiling water and freezing water)
        ShowFloatValue(T1);
        
        
        T2 = 0;
//        analog=read_adc(6);
//        if (analog > 200) status |= 0b00000001;
//        else if ((analog > 112)&&(analog < 140)) status |= 0b00000001;
//        else status &= 0b11111110; //--> door is open

    if (read_adc(6) < 150) status |= 0b00000010; else status &= 0b11111101; //--> emergency button pressed  
    if (read_adc(7) > 150) status |= 0b00000001; else status &= 0b11111110; //--> door is open  

//        if (analog < 138) status |= 0b00000010; else status &= 0b11111101; //--> emergency button pressed

        Report();

        delay_ms(100);
    }
}

void ShowValue(signed char val) {
    unsigned char dig2, dig3;

    if (val < 0) _7seg1G=0; else _7seg1G=1;
    if (val < 0) val = 0-val;
    
    dig2 = val / 10;
    dig3 = val % 10; 

    switch (dig2) {
        case 0: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=0; _7seg2E=0; _7seg2F=0; _7seg2G=1; _7seg2dp=1; break;
        case 1: _7seg2A=1; _7seg2B=0; _7seg2C=0; _7seg2D=1; _7seg2E=1; _7seg2F=1; _7seg2G=1; _7seg2dp=1; break;
        case 2: _7seg2A=0; _7seg2B=0; _7seg2C=1; _7seg2D=0; _7seg2E=0; _7seg2F=1; _7seg2G=0; _7seg2dp=1; break;
        case 3: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=0; _7seg2E=1; _7seg2F=1; _7seg2G=0; _7seg2dp=1; break;
        case 4: _7seg2A=1; _7seg2B=0; _7seg2C=0; _7seg2D=1; _7seg2E=1; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
        case 5: _7seg2A=0; _7seg2B=1; _7seg2C=0; _7seg2D=0; _7seg2E=1; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
        case 6: _7seg2A=0; _7seg2B=1; _7seg2C=0; _7seg2D=0; _7seg2E=0; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
        case 7: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=1; _7seg2E=1; _7seg2F=1; _7seg2G=1; _7seg2dp=1; break;
        case 8: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=0; _7seg2E=0; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
        case 9: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=0; _7seg2E=1; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
    }
    
    switch (dig3) {
        case 0: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=0; _7seg3E=0; _7seg3F=0; _7seg3G=1; break;
        case 1: _7seg3A=1; _7seg3B=0; _7seg3C=0; _7seg3D=1; _7seg3E=1; _7seg3F=1; _7seg3G=1; break;
        case 2: _7seg3A=0; _7seg3B=0; _7seg3C=1; _7seg3D=0; _7seg3E=0; _7seg3F=1; _7seg3G=0; break;
        case 3: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=0; _7seg3E=1; _7seg3F=1; _7seg3G=0; break;
        case 4: _7seg3A=1; _7seg3B=0; _7seg3C=0; _7seg3D=1; _7seg3E=1; _7seg3F=0; _7seg3G=0; break;
        case 5: _7seg3A=0; _7seg3B=1; _7seg3C=0; _7seg3D=0; _7seg3E=1; _7seg3F=0; _7seg3G=0; break;
        case 6: _7seg3A=0; _7seg3B=1; _7seg3C=0; _7seg3D=0; _7seg3E=0; _7seg3F=0; _7seg3G=0; break;
        case 7: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=1; _7seg3E=1; _7seg3F=1; _7seg3G=1; break;
        case 8: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=0; _7seg3E=0; _7seg3F=0; _7seg3G=0; break;
        case 9: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=0; _7seg3E=1; _7seg3F=0; _7seg3G=0; break;
    }
    
}

void ShowFloatValue(float val) {
    signed int sitemp;
    
    sitemp = (val * 10);
    if ((sitemp > 99) || (sitemp < -99)) {
        ShowValue (sitemp / 10);
        return;
    }            
    
    ShowValue (sitemp);
    _7seg2dp=0;

}

void Report(void) {
    unsigned char buff[20], crc, i;
    unsigned char d1, d2, d3;
    unsigned int uitemp;
    float ftemp;

    format_converter_32.f = T1;
    buff[0] = format_converter_32.ui8[0];
    buff[1] = format_converter_32.ui8[1];
    buff[2] = format_converter_32.ui8[2];
    buff[3] = format_converter_32.ui8[3];
                    
    format_converter_32.f = T2;
    buff[4] = format_converter_32.ui8[0];
    buff[5] = format_converter_32.ui8[1];
    buff[6] = format_converter_32.ui8[2];
    buff[7] = format_converter_32.ui8[3];
    buff[8] = status;
    
    crc = 0;
    for (i=0; i<9; i++)
        crc ^= buff[i];
        
    buff[9] = crc;
    
    DataDirection = 1;

    putchar(0x84);
    putchar(0xB3);
    putchar(0xE3);
    putchar(0x75);
    
    for (i=0; i<10; i++)
        putchar(buff[i]);
                    
//    ftemp = T1;
//    buff[0]='+';
//    if (ftemp < 0) {buff[0] = '-'; ftemp = 0 - ftemp;}  
//    uitemp = (ftemp *10);
//    d1 = uitemp / 100;
//    uitemp %= 100;
//    d2 = uitemp / 10;
//    d3 = uitemp %= 10;

//    buff[1] = '0'+d1;
//    buff[2] = '0'+d2;
//    buff[3] = ',';
//    buff[4] = '0'+d3;
    
//    buff[5]=' ';
//
//    ftemp = T2;
//    buff[6]='+';
//    if (ftemp < 0) {buff[6] = '-'; ftemp = 0 - ftemp;}  
//    uitemp = (ftemp *10);
//    d1 = uitemp / 100;
//    uitemp %= 100;
//    d2 = uitemp / 10;
//    d3 = uitemp %= 10;
//
//    buff[7] = '0'+d1;
//    buff[8] = '0'+d2;
//    buff[9] = ',';
//    buff[10] = '0'+d3;
    
//    buff[11]=' ';

//    buff[12] = status;
    
//    crc = 0;
//    for (i=0; i<13; i++)
//        crc ^= buff[i];
//        
//    buff[13] = crc;
//    
//    
//    DataDirection = 1;
//
//    putchar(0x84);
//    putchar(0xB3);
//    putchar(0xE3);
//    putchar(0x75);
//    
//    for (i=0; i<14; i++)
//        putchar(buff[i]);
}

