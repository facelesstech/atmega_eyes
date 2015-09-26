#define F_CPU 16000000UL // 16MHz clock
 
#define CTC_MATCH_OVERFLOW ((F_CPU / 1000) / 8) // Calculate the value needed for the CTC match value in OCR1A
#define	ATOMIC_BLOCK

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Pins for the led matrix
#define CLK_HIGH()  PORTB |= (1<<PB2)
#define CLK_LOW()   PORTB &= ~(1<<PB2)
#define CS_HIGH()   PORTB |= (1<<PB1)
#define CS_LOW()    PORTB &= ~(1<<PB1)
#define DATA_HIGH() PORTB |= (1<<PB0)
#define DATA_LOW()  PORTB &= ~(1<<PB0)
#define INIT_PORT() DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2)

int buttonPushCounter = 1; // Keeps count of the button pushes

// Timer amounts
volatile unsigned long timerMillis1;
long eyeSince1 = 0;
long eyeSince2 = 1000;
long eyeSince3 = 2000;
long eyeSince4 = 3000;
long eyeWaitTime = 4000;

long heartSince1 = 0;
long heartSince2 = 1000;
long heartWaitTime = 2000;

long rotateSince1 = 0;
long rotateSince2 = 10000;
long rotateWaitTime = 20000;

int eyeRotate = 0;

const __flash uint8_t eye[8] = {
        0b00000000,
        0b00111100,
        0b01111110,
        0b01100110,
        0b01100110,
        0b01111110,
        0b00111100,
        0b0000000};

const __flash uint8_t eye1[8] = {
        0b00000000,
        0b00111100,
        0b01111110,
        0b01001110,
        0b01001110,
        0b01111110,
        0b00111100,
        0b0000000};

const __flash uint8_t eye2[8] = {
        0b00000000,
        0b00111100,
        0b01111110,
        0b01110010,
        0b01110010,
        0b01111110,
        0b00111100,
        0b0000000};

const __flash uint8_t bigHeart[8] = {
        0b01100110,
        0b11111111,
        0b11111111,
        0b11111111,
        0b01111110,
        0b00111100,
        0b00011000,
        0b00000000};
          
const __flash uint8_t smallHeart[8] = {
        0b00000000,
        0b00100100,
        0b01111110,
        0b01111110,
        0b00111100,
        0b00011000,
        0b00000000,
        0b00000000};

//const __flash uint8_t bigHeart[8] = {
//        0b11111111,
//        0b10000001,
//        0b10000001,
//        0b10000001,
//        0b10000001,
//        0b10000001,
//        0b10000001,
//        0b11111111};
//
//          
//const __flash uint8_t smallHeart[8] = {
//        0b00000000,
//        0b00100100,
//        0b01111110,
//        0b01111110,
//        0b00111100,
//        0b00011000,
//        0b00000000,
//        0b00000000};
void setColor(int red, int green, int blue)
{
  OCR2B = 255 - red; // Set PWM rate 0 - 255 RED
  OCR0B = 255 - green; // Set PWM rate 0 - 255 GREEN
  OCR0A = 255 - blue; // Set PWM rate 0 - 255 BLUE
}

void spiSend(uint8_t data)
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

void max7219Writec(uint8_t highByte, uint8_t lowByte)
{
    CS_LOW();
    spiSend(highByte);
    spiSend(lowByte);
    CS_HIGH();
}

void max7219Clear(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
	max7219Writec(i+1, 0);
    }
}

void max7219Init(void)
{
    INIT_PORT();
    // Decode mode: none
    max7219Writec(0x09, 0);
    // Intensity: 3 (0-15)
    max7219Writec(0x0A, 1);
    // Scan limit: All "digits" (rows) on
    max7219Writec(0x0B, 7);
    // Shutdown register: Display on
    max7219Writec(0x0C, 1);
    // Display test: off
    max7219Writec(0x0F, 0);
    max7219Clear();
}


uint8_t display[8];

void updateDisplay(void)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
	max7219Writec(i+1, display[i]);
    }
}

void image(const __flash uint8_t im[8])
{
    uint8_t i;

    for (i = 0; i < 8; i++)
	display[i] = im[i];
}

void setPixel(uint8_t r, uint8_t c, uint8_t value)
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

  millisReturn = timerMillis1;
 
  return millisReturn;
}

void eyeHeart ()
{
  if (millis() >= rotateSince1) {
    eyeRotate = 0;
    rotateSince1 += rotateWaitTime;
    }

  if (millis() >= rotateSince2) {
    eyeRotate = 1; 
    rotateSince2 += rotateWaitTime;
    }
}

void eyeDisplay ()
{
  if (eyeRotate == 0)
  {
    if (millis() >= eyeSince1) {
      image(eye); // Load image
      updateDisplay(); // Display on led
      eyeSince1 += eyeWaitTime;
      }

    if (millis() >= eyeSince2) {
      image(eye1); // Load image
      updateDisplay(); // Display on led
      eyeSince2 += eyeWaitTime;
      }

    if (millis() >= eyeSince3) {
      image(eye); // Load image
      updateDisplay(); // Display on led
      eyeSince3 += eyeWaitTime;
      }

    if (millis() >= eyeSince4) {
      image(eye2); // Load image
      updateDisplay(); // Display on led
      eyeSince4 += eyeWaitTime;
      }
  }

  if (eyeRotate == 1)
  {
    if (millis() >= heartSince1) {
      image(bigHeart); // Load image
      updateDisplay(); // Display on led
      heartSince1 += heartWaitTime;
      }

    if (millis() >= heartSince2) {
      image(smallHeart); // Load image
      updateDisplay(); // Display on led
      heartSince2 += heartWaitTime;
      }
  }
}
 

int Pressed = 0;
int pressedConfidenceLevel = 0; //Measure button press cofidence
int releasedConfidenceLevel = 0; //Measure button release confidence

void button(void)
{
    if (bit_is_clear(PIND, 4)) // PD4
    {
      pressedConfidenceLevel ++; //Increase Pressed Conficence
      releasedConfidenceLevel = 0; //Reset released button confidence since there is a button press
      if (pressedConfidenceLevel >500) //Indicator of good button press
      {
        if (Pressed == 0)
        {
          // Do stuff here
          PORTD &= ~_BV(PORTD7); // Candle LED off
          buttonPushCounter++;
          if (buttonPushCounter == 11) {
            buttonPushCounter = 1;
            }

          Pressed = 1;
        }
        //Zero it so a new pressed condition can be evaluated
        pressedConfidenceLevel = 0;
      }
    }

    else
    {
      releasedConfidenceLevel++; //This works just like the pressed
      pressedConfidenceLevel = 0; //Reset pressed button confidence since the button is released
      if (releasedConfidenceLevel >500)
      {
        Pressed = 0;
        releasedConfidenceLevel = 0;
      }
    }
}

void colourChange(void)
{

  if (buttonPushCounter == 1) {
    setColor(255, 255, 0); // YELLOW
    }

  if (buttonPushCounter == 2) {
    setColor(255, 127, 0);  // Orange
    }

  if (buttonPushCounter == 3) {
    setColor(255, 0, 0); // RED
    }

  if (buttonPushCounter == 4) {
    setColor(175, 0, 175);  // Purple
    }

  if (buttonPushCounter == 5) {
    setColor(0, 255, 255);  // Aqua
    }

  if (buttonPushCounter == 6) {
    setColor(0, 0, 255); // BLUE
    }

  if (buttonPushCounter == 7) {
    setColor(0, 255, 0); // GREEN
    }
    
  if (buttonPushCounter == 8) {
    setColor(255, 255, 255); // WHITE 
    }

  if (buttonPushCounter == 9) {
    PORTD |= _BV(PORTD7); // Candle LED on
    setColor(0, 0, 0); // OFF
    }
    
  if (buttonPushCounter == 10) {
    setColor(0, 0, 0); // OFF
    }
}


int main(void)
{
  DDRD |= 1 << PD7; // Set PD7/pin13 as output
  DDRD &= ~(1 << PD4); // Set PD4/pin6 as input
  PORTD |= 1 << PD4; // Set PD4/pin6 as input

  DDRD |= (1 << PD3); // PD3/pin5 is now an output
  DDRD |= (1 << PD5); // PD5/pin11 is now an output
  DDRD |= (1 << PD6); // PD6/pin12 is now an output
  
  // PWM timers for the RGB LED's
  TCCR2A =  (1 << COM2B1) | (1<<COM0A1) | (1<<WGM00) | (1 << WGM20);
  TCCR2B = (1 << CS22) | (1<<CS01);
  TCCR0A = (1<<WGM00) | (1<<COM0A1) | (1 << COM2B1) | (1 << WGM20);
  TCCR0B = (1<<CS01);


  TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, Clock/8
 
  OCR1AH = (CTC_MATCH_OVERFLOW >> 8); // Load the high byte, then the low byte into the output compare
  OCR1AL = (unsigned char) CTC_MATCH_OVERFLOW; // Added (unsigned char)

  TIMSK1 |= (1 << OCIE1A); // Enable the compare match interrupt

  sei(); // Now enable global interrupts
    
  //uint8_t i;
  
  max7219Init(); // Start the 7xxx logic
  while (1)
  {
  button();
  eyeDisplay();
  eyeHeart();
  colourChange();
  }
}

