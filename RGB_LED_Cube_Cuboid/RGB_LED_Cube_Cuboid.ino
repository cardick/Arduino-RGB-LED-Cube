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
#define CLOCK_PIN 13
// must be 11 defined by SPI
#define DATA_PIN 11
#define LATCH_PIN 2
#define BLANK_PIN 4

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
#define LED_ROWS 1
#define LED_COLUMNS 4
#define LED_LAYERS 4

// 4 bit - bit angle modulation (BAM)
#define BAM 4

// maximum and minimum brightness for LEDs
#define MAXBRIGHT 15
#define MINBRIGHT 0

/**
 * Direction flags for the base directions.
 */
enum Direction {
  Zero = 0 << 0,
  Front = 1 << 0,
  Left = 1 << 1,
  Up = 1 << 2,
  Back = 1 << 3,
  Right = 1 << 4,
  Down = 1 << 5
};

/**
 * Structure for a 3D vector that indicates a point or a movement direction in 
 * the cubes or cuboid space. 
 */
typedef struct {
  int x;
  int y;
  int z;
} Vector3D;

// bit mask for the led state information (LED_COLUMS * 3, because each LED has 
// three cathode pins).
bool ledCubeBitMask[BAM][LED_ROWS][LED_COLUMNS * 3][LED_LAYERS];

// This variables are used to shift out data to the shift registers
uint8_t currentTick = 0;
uint8_t currentLayer = 0;


void setup() {
  // disable interrupts
  cli();

  SPI.begin();

  SPI.setBitOrder(MSBFIRST);  // most significant bit first vs. least significant bit first
  // Set LSB as bit order, this means we will send the least significant bit first and it
  // will be written to Q7 = Register Pin 7 with the most significant bit being written last
  // to Q0 or Pin 15

  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);

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
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(BLANK_PIN, OUTPUT);

  // clear all leds
  clearLeds();

  // allow interrupts
  sei();

  // start serial connection - for debugging reasons
  Serial.begin(9600);
}
uint8_t currentCorner = 0;

/**
* Within the loop only the bytes should be manipulated that are written out in ISR method
*/
void loop() {
  Vector3D corner, dir;
  setDirection(&dir, random()%64);
  setCornerCoordinates(&corner, &dir);

  while(isZeroVector(&dir) || !canMove(&corner, &dir)){
    setDirection(&dir, random()%64);
  }
  drawLine(&corner, &dir);

  // snake(30000, millis(), 80);
  delay(200);
  clearLeds();
}

/**
 *
 */
void printVector3D(String name, Vector3D * vec) {
  Serial.print(name);
  Serial.print("(");
  Serial.print(vec->x);
  Serial.print(",");
  Serial.print(vec->y);
  Serial.print(",");
  Serial.print(vec->z);
  Serial.println(")");
}

/**
 * Sets the coordinates of a corner of the cube to the vector.
 *
 * @param vector The vector that should represent the corner.
 * @param direction The direction vector that points to the corner.   
 */
void setCornerCoordinates(Vector3D * vector, const Vector3D * direction) {
    vector -> x = direction -> x <= 0 ? 0 : LED_ROWS-1;
    vector -> y = direction -> y <= 0 ? 0 : LED_COLUMNS-1;
    vector -> z = direction -> z <= 0 ? 0 : LED_LAYERS-1;
}

/**
 * Set direction coordinates to a vector. Can be used to set a direction by 
 * using the Direction enum (e.g. Left|Up, Right, Down|Back, Right|Up|Front).
 *
 * @param vector - the vector to set the coordinates
 * @param direction - the direction to set
 */
void setDirection(Vector3D * vector, Direction direction) {
  setDirection(vector, (uint8_t)direction);
}

/**
 * Set the direction coordinates to a vector. Could be used to get a random 
 * direction. 
 * 
 * @param vector - the vector to set the coordinates
 * @param direction - unless there are 6 base directions the max value is
 *    maximum value of 6 bits (63)  
 */
void setDirection(Vector3D * vector, uint8_t direction) {
  direction = constrain(direction, 0, 63);

  // zero vector - no direction
  vector -> x = 0;
  vector -> y = 0;
  vector -> z = 0;

  // set directions to vector 
  for (uint8_t i = 0; i < 6; i++) {
    switch (direction & (1 << i)) {
      case Front:
      vector -> x += 1; 
      break;
      case Left:
      vector -> y += 1;
      break;
      case Up:
      vector -> z += 1;
      break;
      case Back:
      vector -> x -= 1; 
      break;
      case Right:
      vector -> y -= 1;
      break;
      case Down:
      vector -> z -= 1;
      break;
    }
  }
}

/**
 * 
 */
void inverse(Vector3D * vector) {
  vector->x = vector->x * (-1);
  vector->y = vector->y * (-1);
  vector->z = vector->z * (-1);
} 

/**
 * Draw a line from a point in the cube into the directon given by a direction 
 * vecor.
 *
 * @param point - the initial point (LED) in the cube
 * @param direction - the direction to draw through
 */
void drawLine(Vector3D * point, const Vector3D * direction) {
  while (canMove(point, direction)) {
    setLed(point->x, point->y, point->z, MAXBRIGHT, MAXBRIGHT, MAXBRIGHT);
    point->x += direction->x;
    point->y += direction->y;
    point->z += direction->z;
    delay(60);
  }
  setLed(point->x, point->y, point->z, MAXBRIGHT, MAXBRIGHT, MAXBRIGHT);
}

bool canMove(const Vector3D * point, const Vector3D * direction) {
  if(point->x + direction->x < 0 || point->x + direction->x > LED_ROWS-1) {
    return false;
  }
  if(point->y + direction->y < 0 || point->y + direction->y > LED_COLUMNS-1) {
    return false;
  }
  if(point->z + direction->z < 0 || point->z + direction->z > LED_LAYERS-1) {
    return false;
  }
  return true;
}

/**
 * Indicats whether the given vector is the zero vector (0,0,0).
 */
bool isZeroVector(const Vector3D * vec) {
  return vec->x == 0 && vec->y == 0 && vec->z == 0; 
}

/**
 * Snake animation
void snake(unsigned long runTimeInMillis, unsigned long startTimeInMillis, unsigned long speedInMillis) {
  int x = random() % LED_ROWS;
  int y = random() % LED_COLUMNS;
  int z = random() % LED_LAYERS;
  int lx, ly, lz = -1;

  int lastDir = 0;
  int dirCtr = 0;

  while ((millis() - startTimeInMillis) < runTimeInMillis) {
    clearLeds();

    // get direction
    int direction = random() % 6;

    //dont't go directly back and igonore other rows
    while (lastDir + 3 == direction || direction == FRONT || direction == BACK) {
      direction = random() % 6;
    }

    // count steps to same direction
    if (lastDir == direction) {
      dirCtr++;
    }

    // don't go more than 3 steps into the same direction
    if (dirCtr == 3) {
      //dont't go directly back and cand igonore other rows
      while (lastDir + 3 == direction || lastDir == direction || direction == FRONT || direction == BACK) {
        direction = random() % 6;
      }
      dirCtr = 0;
    }
    lastDir = direction;

    if (lx >= 0) {
      setLed(lx, ly, lz, 1, 1, MINBRIGHT);
    }
    lx = x;
    ly = y;
    lz = z;

    setLed(x, y, z, 2, 2, MINBRIGHT);

    // mem direction
    x = constrain(x + DIRECTION_VECTOR[direction][0], 0, LED_ROWS - 1);
    y = constrain(y + DIRECTION_VECTOR[direction][1], 0, LED_COLUMNS - 1);
    z = constrain(z + DIRECTION_VECTOR[direction][2], 0, LED_LAYERS - 1);

    // new LED on
    setLed(x, y, z, MAXBRIGHT, MAXBRIGHT, MINBRIGHT);
    delay(speedInMillis);
  }
}
*/

void fadingTest() {
  // fade in
  for (int i = 0; i < 12; i++) {
    for (int j = 0; j < LED_COLUMNS; j++) {
      setLed(0, j, 0, i, i, 0);
      setLed(0, j, 1, 0, i, i / 2);
    }
    delay(120);
  }

  delay(1000);

  // fade bottom one from red to blue
  for (int i = 0; i < 12; i++) {
    for (int j = 0; j < LED_COLUMNS; j++) {
      setLed(0, j, 0, 12 - i, 12, i);
    }
    delay(120);
  }

  delay(1000);
  /*
  // fade color from upper green to lower red
  for (int i = 0; i < 12; i++) {
    setLed(0, 0, 0, 0, 12 - i, 12);
    setLed(0, 0, 1, i, 12, 6);
    delay(120);
  }

  delay(1000);
*/
  // fade all out, and see if it crashes
  for (int i = 0; i < 12; i++) {
    for (int j = 0; j < LED_COLUMNS; j++) {
      setLed(0, j, 0, 0, 0, 12 - i);
      setLed(0, j, 1, 12 - i, 12 - i, 6 - (i / 2));
    }
    delay(120);
  }

  // set off
  for (int i = 0; i < LED_COLUMNS; i++) {
    setLed(0, i, 0, 0, 0, 0);
    setLed(0, i, 1, 0, 0, 0);
  }
  delay(1000);
}

// #########################
// the driver specific stuff
// #########################

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
  // correct modulo 8 is 0
  shift = shift == -1 ? 7 : shift;

  // The idea is to shift out the led information from back to front from the perspective of the
  // daisy chained shift registers. Shift out only the current layer for the current tick given by
  // the BAM duty cycle. The rest is done by Arduino by repetetive calls of this method.
  for (int j = (LED_ROWS - 1); j >= 0; j--) {
    for (int k = ((LED_COLUMNS * 3) - 1); k >= 0; k--) {
      transferByte = transferByte | (ledCubeBitMask[bamPos(currentTick)][j][k][currentLayer] << shift);
      shift--;

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
      //shift out the byte
      SPI.transfer(transferByte);

      //reset transfer byte and shift
      transferByte = 0b00000000;
      shift = 7;
    }
  }

  // set latch low than high to activate shift registers
  // PORTD &= 0 << LATCH_PIN;
  // PORTD |= 1 << LATCH_PIN;
  PORTD |= 1 << LATCH_PIN;
  PORTD &= ~(1 << LATCH_PIN);
  PORTD &= ~(1 << BLANK_PIN);

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

