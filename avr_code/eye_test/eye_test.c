#define F_CPU 16000000UL // 16MHz clock 

#define CTC_MATCH_OVERFLOW ((F_CPU / 1000) / 8) // Calculate the value needed for the CTC match value in OCR1A
#define	ATOMIC_BLOCK

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Pins for the LED matrix, Can be chained together
#define CLK_HIGH()  PORTB |= (1<<PB2)
#define CLK_LOW()   PORTB &= ~(1<<PB2)
#define CS_HIGH()   PORTB |= (1<<PB1)
#define CS_LOW()    PORTB &= ~(1<<PB1)
#define DATA_HIGH() PORTB |= (1<<PB0)
#define DATA_LOW()  PORTB &= ~(1<<PB0)
#define INIT_PORT() DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2)

// Wait times
volatile unsigned long timerMillis1;
long heartSince1 = 0;
long heartSince2 = 500;
long heartWaitTime = 1000;

// old eye
//const __flash uint8_t animationOne[8] = {
//        0b00000000,
//        0b00111100,
//        0b01111110,
//        0b01110010,
//        0b01110010,
//        0b01111110,
//        0b00111100,
//        0b0000000};
//
//const __flash uint8_t animationTwo[8] = {
//        0b00000000,
//        0b00111100,
//        0b01111110,
//        0b01100110,
//        0b01100110,
//        0b01111110,
//        0b00111100,
//        0b0000000};

// cats eyes
//const __flash uint8_t animationOne[8] = {
//        0b00000000,
//        0b00111100,
//        0b01100110,
//        0b11100111,
//        0b11100111,
//        0b01100110,
//        0b00111100,
//        0b0000000};
//
//const __flash uint8_t animationTwo[8] = {
//        0b00000000,
//        0b00111100,
//        0b01110010,
//        0b11110011,
//        0b11110011,
//        0b01110010,
//        0b00111100,
//        0b0000000};

// big heart
//const __flash uint8_t animationOne[8] = {
//        0b01100110,
//        0b11111111,
//        0b11111111,
//        0b11111111,
//        0b01111110,
//        0b00111100,
//        0b00011000,
//        0b00000000};
//
//// little heart
//const __flash uint8_t animationTwo[8] = {
//        0b00000000,
//        0b00100100,
//        0b01111110,
//        0b01111110,
//        0b00111100,
//        0b00011000,
//        0b00000000,
//        0b00000000};
          
// chevrons
//const __flash uint8_t animationOne[8] = {
//        0b10001000,
//        0b01000100,
//        0b00100010,
//        0b00010001,
//        0b00100010,
//        0b01000100,
//        0b10001000,
//        0b0000000};
//          
//// chevrons
//const __flash uint8_t animationTwo[8] = {
//        0b01000100,
//        0b00100010,
//        0b00010001,
//        0b00001000,
//        0b00010001,
//        0b00100010,
//        0b01000100,
//        0b00000000};

// new eyes
//const __flash uint8_t animationOne[8] = {
//        0b00000000,
//        0b01111100,
//        0b11111110,
//        0b11000110,
//        0b11010110,
//        0b11000110,
//        0b01111100,
//        0b0000000};
//          
//const __flash uint8_t animationTwo[8] = {
//        0b00000000,
//        0b01111100,
//        0b11111110,
//        0b11100010,
//        0b11101010,
//        0b11100010,
//        0b01111100,
//        0b0000000};

// Skull
const __flash uint8_t animationOne[8] = {
        0b01111110,
        0b11111111,
        0b10011001,
        0b11011101,
        0b11111111,
        0b01111110,
        0b01011010,
        0b00000000};

const __flash uint8_t animationTwo[8] = {
        0b01111110,
        0b11111111,
        0b10011001,
        0b10111011,
        0b11111111,
        0b01111110,
        0b01011010,
        0b00000000};

void spi_send(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++, data <<= 1)
    {
	CLK_LOW();
	if (data & 0x80)
	    DATA_HIGH();
	else
	    DATA_LOW();
	CLK_HIGH();
    }
    
}

void max7219_writec(uint8_t high_byte, uint8_t low_byte)
{
    CS_LOW();
    spi_send(high_byte);
    spi_send(low_byte);
    CS_HIGH();
}

void max7219_clear(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
	max7219_writec(i+1, 0);
    }
}

void max7219_init(void)
{
    INIT_PORT();
    // Decode mode: none
    max7219_writec(0x09, 0);
    // Intensity: 3 (0-15)
    max7219_writec(0x0A, 1);
    // Scan limit: All "digits" (rows) on
    max7219_writec(0x0B, 7);
    // Shutdown register: Display on
    max7219_writec(0x0C, 1);
    // Display test: off
    max7219_writec(0x0F, 0);
    max7219_clear();
}


uint8_t display[8];

void updateDisplay(void)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
	max7219_writec(i+1, display[i]);
    }
}

void image(const __flash uint8_t im[8])
{
    uint8_t i;

    for (i = 0; i < 8; i++)
	display[i] = im[i];
}

void set_pixel(uint8_t r, uint8_t c, uint8_t value)
{
    switch (value)
    {
    case 0: // Clear bit
	display[r] &= (uint8_t) ~(0x80 >> c);
	break;
    case 1: // Set bit
	display[r] |= (0x80 >> c);
	break;
    default: // XOR bit
	display[r] ^= (0x80 >> c);
	break;
    }
}

ISR (TIMER1_COMPA_vect)
{
    timerMillis1++;
}

unsigned long millis()
{
  unsigned long millisReturn;

  ATOMIC_BLOCK(ATOMIC_FORCEON) // Ensure this cannot be disrupted
  { 
  millisReturn = timerMillis1;
  }
 
  return millisReturn;
}

void eyeDisplay ()
{
  if (millis() >= heartSince1) {
    image(animationOne); // Load image
    updateDisplay(); // Display on led
    heartSince1 += heartWaitTime;
    }

  if (millis() >= heartSince2) {
    image(animationTwo); // Load image
    updateDisplay(); // Display on led
    heartSince2 += heartWaitTime;
    }
}


int main(void)
{
  TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, Clock/8
 
  OCR1AH = (CTC_MATCH_OVERFLOW >> 8); // Load the high byte, then the low byte into the output compare
  OCR1AL = (unsigned char) CTC_MATCH_OVERFLOW; // Added (unsigned char)

  TIMSK1 |= (1 << OCIE1A); // Enable the compare match interrupt

  sei(); // Now enable global interrupts
  max7219_init();
    
  while(1)
  {
    eyeDisplay();
  }
}
