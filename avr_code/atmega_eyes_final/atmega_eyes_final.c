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

int buttonPushCounter = 8; // Keeps count of the button pushes

// Timer amounts
volatile unsigned long timerMillis1;
long blinkSince1 = 0;
long blinkSince2 = 50;
long blinkSince3 = 100;
long blinkSince4 = 150;
long blinkWaitTime = 200;

long lookSince1 = 0;
long lookSince2 = 150;
long lookSince3 = 300;
long lookSince4 = 800;
long lookSince5 = 950;
long lookSince6 = 1100;
long lookSince7 = 1250;
long lookSince8 = 1750;
long lookWaitTime = 1750;

long scrollSince1 = 0;
long scrollSince2 = 150;
long scrollSince3 = 300;
long scrollSince4 = 450;
long scrollSince5 = 600;
long scrollSince6 = 750;
long scrollSince7 = 900;
long scrollSince8 = 1050;
long scrollWaitTime = 1200;

long rollSince1 = 0;
long rollSince2 = 150;
long rollSince3 = 300;
long rollSince4 = 450;
long rollSince5 = 600;
long rollSince6 = 900;
long rollSince7 = 1050;
long rollSince8 = 1200;
long rollWaitTime = 1200;

long blinkRotateSince1 = 3900;
long blinkRotateSince2 = 100;
long blinkRotateWaitTime = 4000;

long rotateSince1 = 10000;
long rotateWaitTime = 10000;

int eyeRotate = 0;
int blinkRotate = 0;

// RGB 
int rgbRed = 255;  
int rgbBlue = 0;  
int rgbGreen = 255;
int move = 0;
// RGB timers
long rainbowSince1 = 0;
long rainbowWaitTime = 10; 

// Blink 1
const __flash uint8_t animationTwo[8] = {
        0b00000000,
        0b00111100,
        0b01111110,
        0b11111111,
        0b01111110,
        0b00111100,
        0b00000000,
        0b00000000};

// Blink 2
const __flash uint8_t animationThree[8] = {
        0b00000000,
        0b00000000,
        0b00111100,
        0b01111110,
        0b00111100,
        0b00000000,
        0b00000000,
        0b00000000};

// Blank 3
const __flash uint8_t animationFour[8] = {
        0b00000000,
        0b00000000,
        0b00000000,
        0b00111100,
        0b00000000,
        0b00000000,
        0b00000000,
        0b00000000};

// Eye 1
const __flash uint8_t animationLookHome[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11100111,
        0b11100111,
        0b11111111,
        0b11111111,
        0b00000000};
 
// Eye 2
const __flash uint8_t animationLook1[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11001111,
        0b11001111,
        0b11111111,
        0b11111111,
        0b00000000};
// Eye 3
const __flash uint8_t animationLook2[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b10011111,
        0b10011111,
        0b11111111,
        0b11111111,
        0b00000000};

// Eye 4
const __flash uint8_t animationLook3[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11001111,
        0b11001111,
        0b11111111,
        0b00000000};

// Eye 5 
const __flash uint8_t animationLook4[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11100111,
        0b11100111,
        0b11111111,
        0b00000000};

// Eye 6
const __flash uint8_t animationLook5[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11110011,
        0b11110011,
        0b11111111,
        0b00000000};

// Eye 7
const __flash uint8_t animationLook6[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111001,
        0b11111001,
        0b11111111,
        0b11111111,
        0b00000000};

// Eye 8 
const __flash uint8_t animationLook7[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11110011,
        0b11110011,
        0b11111111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationLook8[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111101,
        0b11111101,
        0b11111111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationBlank[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationLook10[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b10111111,
        0b10111111,
        0b11111111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationRoll1[8] = {
        0b11111111,
        0b11111111,
        0b11100111,
        0b11100111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationRoll2[8] = {
        0b11111111,
        0b11100111,
        0b11100111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationRoll3[8] = {
        0b11111111,
        0b11100111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationRoll4[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11100111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationRoll5[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11111111,
        0b11100111,
        0b11100111,
        0b11111111,
        0b00000000};

const __flash uint8_t animationRoll6[8] = {
        0b11111111,
        0b11111111,
        0b11111111,
        0b11100111,
        0b11100111,
        0b11111111,
        0b11111111,
        0b00000000};

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

  ATOMIC_BLOCK(ATOMIC_FORCEON) // Ensure this cannot be disrupted
  { 
  millisReturn = timerMillis1;
  }
 
  return millisReturn;
}

void eyeSwitchBlink ()
{
  if (millis() >= blinkRotateSince1) 
  {
    blinkRotate = 0;
    blinkRotateSince1 += blinkRotateWaitTime;
  }
  
  if (millis() >= blinkRotateSince2)
  {
    blinkRotate = 1;
    blinkRotateSince2 += blinkRotateWaitTime;
  }
}

void eyeSwitch ()
{
  if (millis() >= rotateSince1) 
  {
    eyeRotate++;
    if (eyeRotate == 3) 
    {
      eyeRotate = 0;
    }
    rotateSince1 += rotateWaitTime;
  }
}

void eyeBlink ()
{
  if (millis() >= blinkSince1) 
  {
    image(animationTwo); // Load image
    updateDisplay(); // Display on led
    blinkSince1 += blinkWaitTime;
  }

  if (millis() >= blinkSince2) 
  {
    image(animationThree); // Load image
    updateDisplay(); // Display on led
    blinkSince2 += blinkWaitTime;
  }

  if (millis() >= blinkSince3) 
  {
    image(animationFour); // Load image
    updateDisplay(); // Display on led
    blinkSince3 += blinkWaitTime;
  }

  if (millis() >= blinkSince4) 
  {
    image(animationTwo); // Load image
    updateDisplay(); // Display on led
    blinkSince4 += blinkWaitTime;
  }
}

void eyeDisplay ()
{
  if (eyeRotate == 0)
  {
    if ( blinkRotate == 0)
    {
      eyeBlink();
    }

    if ( blinkRotate == 1)
    {

      if (millis() >= lookSince1) 
      {
        image(animationLookHome); // Load image
        updateDisplay(); // Display on led
        lookSince1 += lookWaitTime;
      }

      if (millis() >= lookSince2) 
      {
        image(animationLook1); // Load image
        updateDisplay(); // Display on led
        lookSince2 += lookWaitTime;
      }

      if (millis() >= lookSince3) 
      {
        image(animationLook2); // Load image
        updateDisplay(); // Display on led
        lookSince3 += lookWaitTime;
      }

      if (millis() >= lookSince4) 
      {
        image(animationLook3); // Load image
        updateDisplay(); // Display on led
        lookSince4 += lookWaitTime;
      }

      if (millis() >= lookSince5) 
      {
        image(animationLook4); // Load image
        updateDisplay(); // Display on led
        lookSince5 += lookWaitTime;
      }

      if (millis() >= lookSince6) 
      {
        image(animationLook5); // Load image
        updateDisplay(); // Display on led
        lookSince6 += lookWaitTime;
      }

      if (millis() >= lookSince7) 
      {
        image(animationLook6); // Load image
        updateDisplay(); // Display on led
        lookSince7 += lookWaitTime;
      }

      if (millis() >= lookSince8) 
      {
        image(animationLook7); // Load image
        updateDisplay(); // Display on led
        lookSince8 += lookWaitTime;
      }
    }
  }
  

  if (eyeRotate == 1)
  {

    if ( blinkRotate == 0)
    {
      eyeBlink();
    }

    if ( blinkRotate == 1)
    {

      if (millis() >= scrollSince1) {
        image(animationLook8); // Load image
        updateDisplay(); // Display on led
        scrollSince1 += scrollWaitTime;
      }

      if (millis() >= scrollSince2) {
        image(animationLook6); // Load image
        updateDisplay(); // Display on led
        scrollSince2 += scrollWaitTime;
      }

      if (millis() >= scrollSince3) 
      {
        image(animationLook7); // Load image
        updateDisplay(); // Display on led
        scrollSince3 += scrollWaitTime;
      }

      if (millis() >= scrollSince4) 
      {
        image(animationLookHome); // Load image
        updateDisplay(); // Display on led
        scrollSince4 += scrollWaitTime;
      }

      if (millis() >= scrollSince5) 
      {
        image(animationLook1); // Load image
        updateDisplay(); // Display on led
        scrollSince5 += scrollWaitTime;
      }

      if (millis() >= scrollSince6) 
      {
        image(animationLook2); // Load image
        updateDisplay(); // Display on led
        scrollSince6 += scrollWaitTime;
      }

      if (millis() >= scrollSince7) 
      {
        image(animationLook10); // Load image
        updateDisplay(); // Display on led
        scrollSince7 += scrollWaitTime;
      }

      if (millis() >= scrollSince8) 
      {
        image(animationBlank); // Load image
        updateDisplay(); // Display on led
        scrollSince8 += scrollWaitTime;
      }
    }
  }

  if (eyeRotate == 2)
  {

    if ( blinkRotate == 0)
    {
      eyeBlink();
    }

    if ( blinkRotate == 1)
    {

      if (millis() >= rollSince1) {
        image(animationLookHome); // Load image
        updateDisplay(); // Display on led
        rollSince1 += rollWaitTime;
      }

      if (millis() >= rollSince2) {
        image(animationRoll1); // Load image
        updateDisplay(); // Display on led
        rollSince2 += rollWaitTime;
      }

      if (millis() >= rollSince3) 
      {
        image(animationRoll2); // Load image
        updateDisplay(); // Display on led
        rollSince3 += rollWaitTime;
      }

      if (millis() >= rollSince4) 
      {
        image(animationRoll3); // Load image
        updateDisplay(); // Display on led
        rollSince4 += rollWaitTime;
      }

      if (millis() >= rollSince5) 
      {
        image(animationBlank); // Load image
        updateDisplay(); // Display on led
        rollSince5 += rollWaitTime;
      }

      if (millis() >= rollSince6) 
      {
        image(animationRoll4); // Load image
        updateDisplay(); // Display on led
        rollSince6 += rollWaitTime;
      }

      if (millis() >= rollSince7) 
      {
        image(animationRoll5); // Load image
        updateDisplay(); // Display on led
        rollSince7 += rollWaitTime;
      }

      if (millis() >= rollSince8) 
      {
        image(animationRoll6); // Load image
        updateDisplay(); // Display on led
        rollSince8 += rollWaitTime;
      }
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
          if (buttonPushCounter == 10) 
          {
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

void rainbow(void)
{
  if (millis() >= rainbowSince1)
  {
    OCR2B =  rgbRed; // red if 0 and rest are 255
    OCR0B =  rgbGreen; // Green if 0 and rest are 255
    OCR0A =  rgbBlue; // Blue if 0 and rest are 255

    if (move == 0) // Blue to Red
    {
      rgbRed--;
      rgbBlue++;

      if (rgbRed == 0 && rgbBlue == 255 && rgbGreen == 255)
      {
        move = 1;
      }
    }

    if (move == 1) // Red to Green
    {
      rgbGreen--;
      rgbRed++;
      if (rgbGreen == 0 && rgbRed == 255 && rgbBlue == 255)
      {
        move = 2;
      }
    }

    if (move == 2) // Green to Blue
    {
      rgbBlue--;
      rgbGreen++;
      if (rgbBlue == 0 && rgbGreen == 255 &&  rgbRed == 255)
      {
        move = 0;
      }
    }

    rainbowSince1 += rainbowWaitTime;
  }
}

void setColor(int red, int green, int blue)
{
  OCR2B = 255 - red; // Set PWM rate 0 - 255 RED
  OCR0B = 255 - green; // Set PWM rate 0 - 255 GREEN
  OCR0A = 255 - blue; // Set PWM rate 0 - 255 BLUE
}

void colourChange(void)
{

  if (buttonPushCounter == 1) 
  {
    setColor(255, 0, 0); // RED
  }

  if (buttonPushCounter == 2) 
  {
    setColor(175, 0, 175);  // Purple
  }

  if (buttonPushCounter == 3) 
  {
    setColor(0, 255, 255);  // Aqua
  }

  if (buttonPushCounter == 4)
  { 
    setColor(0, 0, 255); // BLUE
  }

  if (buttonPushCounter == 5) 
  {
    setColor(0, 255, 0); // GREEN
  }
    
  if (buttonPushCounter == 6) 
  {
    setColor(255, 255, 255); // WHITE 
  }

  if (buttonPushCounter == 7) 
  {
    rainbow(); // run the rgb scroll code
  }
    
  if (buttonPushCounter == 8) 
  {
    PORTD |= _BV(PORTD7); // Candle LED on
    setColor(0, 0, 0); // OFF
  }

  if (buttonPushCounter == 9) 
  {
    setColor(0, 0, 0); // OFF
  }
}

int main(void)
{
  DDRD |= 1 << PD7; // Set PD7/pin13 as output
  DDRD &= ~(1 << PD4); // Set PD4/pin6 as input
  PORTD |= 1 << PD4; // Set PD4/pin6 as input

// RGB leds
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
    
  max7219Init(); // Start the 7xxx logic

  while (1)
  {
  button();
  eyeDisplay();
  eyeSwitch();
  eyeSwitchBlink();
  colourChange();
  }
}

