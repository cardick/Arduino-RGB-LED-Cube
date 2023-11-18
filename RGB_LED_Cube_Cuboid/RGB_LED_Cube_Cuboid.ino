#include <SPI.h>
/**
 * Controll a common anode RGB LED cube or cuboid - inspired by the projects of 
 * Kevin Darrah and Tiemen Waterreus. 
 *
 * The driver methods use SPI to control the shift registers. 
 *
 * @see SPI Tutorial: https://core-electronics.com.au/guides/spi-arduino-tutorial/ 
 * @author Carsten Dick (carsten.dick@googlemail.com)
 */

// must be 13 defined by SPI
const uint8_t clockPin = 13;
// must be 11 defined by SPI
const uint8_t dataPin = 11;

const uint8_t latchPin = 2;
// @todo: Find out for whatever reason Kevin has used this blank pin
const uint8_t blankPin = 4;

// The driver logic is based on rows and columns from the top view onto the cube. 
// LEDs theoretical numbering on a layer is from back left to front right (columns 
// from left to right; rows from back to front; layers from bottom to top).
//
// 2x2x2     4x4x4           ...
// 00 01     00 01 02 03
// 02 04     04 05 06 07
//           08 09 10 11
//           12 13 14 15
//
// The logic also assumes that this is the logic how the LEDs are connected to the
// daisy chained shift registers; each LED with the rgb pins side by side, connected
// in the pin order R-G-B (e.g. LED 00 R-G-B, LED 01 R-G-B and so on).
// Anodes are connected first in the daisy chained shift register; from 0 - n, whereas
// 0 is the anode for the bottom layer.

// Size definition of the cube or cuboid
const uint8_t LED_ROWS = 1;
const uint8_t LED_COLUMNS = 1;
const uint8_t LED_LAYERS = 2;

// 4 bit - bit angle modulation (BAM)
const uint8_t BAM = 4;

// maximum and minimum brightness for LEDs
const uint8_t MAXBRIGHT = 15;
const uint8_t MINBRIGHT = 0;

// bit mask for the led state information (LED_COLUMS * 3, because each LED has three
// cathode pins).
bool ledCubeBitMask[BAM][LED_ROWS][LED_COLUMNS * 3][LED_LAYERS];

// This variables are used to shift out data to the shift registers
uint8_t currentTick = 0;
uint8_t currentLayer = 0;

void setup() {

  //SPI.begin();
  SPI.setBitOrder(MSBFIRST);  // most significant bit first vs. least significant bit first
  // Set LSB as bit order, this means we will send the least significant bit first and it
  // will be written to Q7 = Register Pin 7 with the most significant bit being written last
  // to Q0 or Pin 15
  
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  // disable interrupts
  //cli();
  noInterrupts();

  // initialize the timer interrupts to enable multiplexing

  // set TCCR1A register 
  TCCR1A = B00000000;
  // set TCCR1B register 
  TCCR1B = B00001011;

  // triggers the time of the interrupt
  // an interval faster than 70 seems to mix the colors between layers
  OCR1A = 70;

  TIMSK1 = B00000010;

  // Configure the Arduino pins used as OUTPUT, so they can send data to the Shift Register
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(blankPin, OUTPUT);

  SPI.begin();
  
  // clear all leds
  clearLeds();

  // allow interrupts
  //sei();
  interrupts();

  // start serial connection - for debugging reasons
  // Serial.begin(9600);
}

/**
* Within the loop only the bytes should be manipulated that are written out in ISR method
*/
void loop() {
    // functional check
    setLed(0, 0, 0, MAXBRIGHT, MINBRIGHT, MINBRIGHT);
    setLed(0, 0, 1, MINBRIGHT, MINBRIGHT, MAXBRIGHT);
    delay(500);
    setLed(0, 0, 0, MINBRIGHT, MAXBRIGHT, MINBRIGHT);
    setLed(0, 0, 1, MAXBRIGHT, MINBRIGHT, MINBRIGHT);
    delay(500);
    setLed(0, 0, 0, MINBRIGHT, MINBRIGHT, MAXBRIGHT);
    setLed(0, 0, 1, MINBRIGHT, MAXBRIGHT, MINBRIGHT);
    delay(500);
    setLed(0, 0, 0, MAXBRIGHT, MAXBRIGHT, MAXBRIGHT);
    setLed(0, 0, 1, MAXBRIGHT, MAXBRIGHT, MAXBRIGHT);
    delay(1000);
    setLed(0, 0, 0, MINBRIGHT, MINBRIGHT, MINBRIGHT);
    setLed(0, 0, 1, MINBRIGHT, MINBRIGHT, MINBRIGHT);
    delay(1000);

    fadingTest();
}

void fadingTest(){
  // fade in
  for (int i = 0; i < 12; i++) {
    setLed(0, 0, 0, i, i, 0);
    setLed(0, 0, 1, 0, i, i / 2);
    delay(120);
  }

  delay(1000);

  // fade bottom one from red to blue
  for (int i = 0; i < 12; i++) {
    setLed(0, 0, 0, 12 - i, 12, i);
    delay(120);
  }

  delay(1000);

  // fade color from upper green to lower red
  for (int i = 0; i < 12; i++) {
    setLed(0, 0, 0, 0, 12 - i, 12);
    setLed(0, 0, 1, i, 12, 6);
    delay(120);
  }

  delay(1000);

  // fade all out, and see if it crashes
  for (int i = 0; i < 12; i++) {
    setLed(0, 0, 0, 0, 0, 12 - i);
    setLed(0, 0, 1, 12 - i, 12 - i, 6 - (i / 2));
    delay(120);
  }

  // set off
  setLed(0, 0, 0, 0, 0, 0);
  setLed(0, 0, 1, 0, 0, 0);
  delay(1000);
}

/**
 * Set all LEDs on all brightness levels to zero to turn them off.
 */
void clearLeds() {
  for (int i = 0; i < BAM; i++) {
    for (int j = 0; j < LED_ROWS; j++) {
      for (int k = 0; k < (LED_COLUMNS * 3); k++) {
        for (int l = 0; l < LED_LAYERS; l++) {
          ledCubeBitMask[i][j][k][l] = 0;
        }
      }
    }
  }
}

/**
 * Turn on/off the LED by setting the brightness of the LED for each color.
 *
 * Brightness value is in range from 0 (color is off) to 15 (color with full brightness)
 */
void setLed(uint8_t row, uint8_t column, uint8_t layer, uint8_t red, uint8_t green, uint8_t blue) {

  // ensure row, column and layers are in correct range
  row = constrain(row, 0, LED_ROWS);
  column = constrain(column, 0, LED_COLUMNS);
  layer = constrain(layer, 0, LED_LAYERS);

  // set the led color on/off in given brightness
  setLedColorBrightness(row, (column * 3), layer, red);
  setLedColorBrightness(row, ((column * 3) + 1), layer, green);
  setLedColorBrightness(row, ((column * 3) + 2), layer, blue);
}

/* 
 * #########################
 * the driver specific stuff
 * #########################
 */

/**
 * Map the color brightness to the 4-bit BAM bit mask of the LED.
 *
 * @see 
 * https://medium.com/@tiemenwaterreus/4-bit-angle-modulating-16-leds-using-arduino-and-shift-registers-8b2b738d4ced
 */
void setLedColorBrightness(uint8_t row, uint8_t columnColorPin, uint8_t layer, uint8_t brightness) {
  // ensure 4-bit brightness
  brightness = constrain(brightness, 0b0000, 0b1111);

  // turn 4-bit brightness into cubes brightness mask
  for (int i = 3; i >= 0; i--) {
    if (brightness - (1 << i) >= 0) {
      brightness -= (1 << i);
      ledCubeBitMask[i][row][columnColorPin][layer] = 1;
    } else {
      ledCubeBitMask[i][row][columnColorPin][layer] = 0;
    }
  }
}

/**
 * ISR is the interrupt method executed by the arduino. Multiplexing and BAM is realized here.
 * 
 * @see 
 * Tutorial 1: https://www.simsso.de/?type=arduino/timer-interrupts 
 * Tutorial 2: https://electronoobs.com/eng_arduino_tut140.php 
 * https://medium.com/@tiemenwaterreus/4-bit-angle-modulating-16-leds-using-arduino-and-shift-registers-8b2b738d4ced
 * https://medium.com/@tiemenwaterreus/building-a-4x4x4-led-cube-part-ii-the-software-813a5207bca8
 */
ISR(TIMER1_COMPA_vect) {

  // SPI.transfer is using byte, so we have to shift out the LED cubes BAM bitmask byte wise
  uint8_t transferByte = 0b00000000;

  // there is maybe an offset of unused ports on the shift registers; this depends on the cubes size
  int shift = (((LED_COLUMNS * 3 * LED_ROWS) + LED_LAYERS) % 8) - 1;

// Serial.print("tick: ");
// Serial.println(currentTick);

  // The idea is to shift out the led information from back to front from the perspective of the
  // daisy chained shift registers. Shift out only the current layer for the current tick given by
  // the BAM duty cycle. The rest is done by Arduino by repetetive calls of this method.
  for (int j = (LED_ROWS - 1); j >= 0; j--) {
    for (int k = ((LED_COLUMNS * 3) - 1); k >= 0; k--) {
      transferByte = transferByte | (ledCubeBitMask[bamPos(currentTick)][j][k][currentLayer] << shift--);
      if (shift < 0) {
        // shift out the byte
        SPI.transfer(transferByte);

        // reset transfer byte and shift
        transferByte = 0b00000000;
        shift = 7;
      }
    }
  }

  // Shift out the layer anodes, the logic bases on the idea that the anodes are at the 
  // beginning of the daisy chained shift registers and that the bottem layer is 
  // connected in first position (MSBFIRST)
  for (int i = LED_LAYERS - 1; i >= 0; i--) {
    if (i == currentLayer) {
      transferByte = transferByte | (1 << shift);
    }
    shift--;
    // else {
    //   transferByte = transferByte | (0 << shift--);
    // }

    if (shift < 0) {
      // Serial.print("transfer: ");
      // Serial.println(transferByte, BIN);

      //shift out the byte
      SPI.transfer(transferByte);

      //reset transfer byte and shift
      transferByte = 0b00000000;
      shift = 7;
    }
  }

  // set latch low than high to activate shift registers
  // PORTD &= 0 << latchPin;
  // PORTD |= 1 << latchPin;
  PORTD |= 1 << latchPin;
  PORTD &= ~(1 << latchPin);
  PORTD &= ~(1 << blankPin);

  //Serial.println("latch impuls");

  // reset current layer to the first one, when all layers have been shifted out
  currentLayer = ((currentLayer < LED_LAYERS - 1) ? (currentLayer + 1) : 0);
  
  // when all layers for this tick were shifted out repeat for next tick
  if (currentLayer == 0) {
    // prepare for next tick or restart for next duty cycle
    currentTick = currentTick < maxValue(BAM) ? (currentTick + 1) : 0;
  }
}

/**
 * Returns the maximum position of the tick in the BAM binary sequence (e.g. 0 is with 
 * position 1, 15 is with position 3, 116 is with position 7).
 */
uint8_t bamPos(uint8_t tick) {
  uint8_t bitPos = 1;
  while (tick >= (1 << bitPos)) {
    bitPos++;
  }
  return bitPos - 1;
}

/**
 * Returns the maximum value for a binary sequence of n-bits (e.g. 3 bits have 
 * max value 7, 8 bits have max value 255).
 */
uint8_t maxValue(uint8_t bits) {
  uint8_t bitValue = 0;
  for (uint8_t i = 0; i < bits; i++) {
    bitValue = bitValue | (1 << i);
  }
  return bitValue;
}