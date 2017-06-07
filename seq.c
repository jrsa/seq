#include <avr/io.h>
#include <avr/interrupt.h>

#define SPI_PIN PINB
#define SPI_PORT PORTB
#define SPI_DDR DDRB
#define SPI_MOSI PB3
#define SPI_MISO PB4
#define SPI_SCLK PB5

#define DAC_CS_PORT PORTB
#define DAC_CS_DDR DDRB
#define DAC_CS PB2

#define MCP4921_ABSEL 7
#define MCP4921_BUF 6
#define MCP4921_GAIN 5
#define MCP4921_SHDN 4

#define DEBUG_LED PD0

void int0_init();
void adc_init();
uint16_t adc_read(uint8_t ch);
void dac_init();
void dac_write(uint8_t channel, uint16_t data);

void int0_init() {
    DDRD = (1 << PD2);
    PORTD = (1 << PD2);

    EICRA |= (1 << ISC00) | (1 << ISC01);
    EIMSK |= (1 << INT0);
   
    sei();
}

void dac_init() {
  SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCLK); //SCK, MOSI are output
  DAC_CS_DDR |= (1 << DAC_CS); //CS for MCP4922 DAC
  DAC_CS_PORT |= (1 << DAC_CS); //pull CS high to intialize 
  
  SPI_DDR &= ~(1 << SPI_MISO); // MISO is input
  
  //SPI enable, Master Mode, F_OSC/4, interrupt enabled
  SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR1)| (0 << SPR0); 
  SPSR = (1 << SPI2X); //SPI double speed = 2MHz

  dac_write(0, 0);
  dac_write(1, 0);
}

void adc_init() {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void dac_write(uint8_t channel, uint16_t data) {
  DAC_CS_PORT &= ~(1 << DAC_CS);  //pull CS low to enable DAC
  
  // send configuration bits and highest 4 data bits
  SPDR = (channel<<MCP4921_ABSEL) 
          | (1 << MCP4921_BUF) 
          | (1 << MCP4921_GAIN) 
          | (1 << MCP4921_SHDN) 
          | ((data>>8) & 0x0F);

  while (!(SPSR & (1 << SPIF))) // wait
    ;

  SPDR = data & 0x00FF; // send lo byte

  while (!(SPSR & (1 << SPIF))) // wait
    ;

  DAC_CS_PORT |= (1 << DAC_CS);   //pull CS high to latch data
}
 
uint16_t adc_read(uint8_t ch) {
    ch &= 0b00000110; // limit to 6 channels, atmega8 has 6 adc pins
    ADMUX = (ADMUX & 0xF8) | ch;     // clears the bottom 3 bits before ORing
 
    // start single conversion
    ADCSRA |= (1 << ADSC);
 
    // wait for conversion to complete
    // ADSC becomes '0' again
    // till then, run loop continuously
    while(ADCSRA & (1 << ADSC));
 
    return (ADC);
}

ISR(INT0_vect){
    PIND = (1 << DEBUG_LED); // toggle debug led
}


void main() {
    adc_init();
    dac_init();
    int0_init();

    DDRD |= (1 << DEBUG_LED); // enable debug led output
    
    while(1);
}
