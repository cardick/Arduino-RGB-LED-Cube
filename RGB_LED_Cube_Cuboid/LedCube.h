
/**
 * The cube class is a kind of digital twin of the hardware.
 */
#ifndef LedCube_h
#define LedCube_h

#include <Arduino.h>
#include "LinkedList.h"
#include "Vector3D.h"

/**
 * Brightness flags enabling a 4 Bit BAM (Bit Angle Manipulation).
 */
enum Brightness
{
    Off = 0b0 << 0,
    Low = 0b1 << 0,
    Medium = 0b11 << 0,
    High = 0b111 << 0,
    Full = 0b1111 << 0
};

/**
 * Color struct holds the brightness for each LED color.
 */
typedef struct
{
    Brightness red;
    Brightness green;
    Brightness blue;
} Color;

/**
 * LED is defined by a point in the three dimensional cube and a color setting.
 */
struct LED
{
    Point3D *position;
    Color *color;
};
typedef struct LED LED;

/**
 * Cube class represents the LED cube itself with its characteristics.
 */
class CubeClass
{
public:
    /**
     * Initialize the cube dimension.
     */
    static void init(uint8_t rows, uint8_t cols, uint8_t layers);

    /**
     * Get the amount of rows.
     */
    static uint8_t getRowSize();

    /**
     * Get the amount of columns.
     */
    static uint8_t getColumnSize();

    /**
     * Get the amount of layers.
     */
    static uint8_t getLayerSize();

    /**
     * Set the point to the corner coordinates of the cube in given direction.
     */
    static void setCorner(Point3D *point, const Vector3D *direction);

    /**
     * Print short summary of the settings of the cube to Serial.
     */
    static void print();

    /**
     * Print the state of the cubes LEDs in JSON format to Serial.
     */
    static void printJsonLeds();

    /**
     * Print the state of a LED to Serial.
     * 
     * @param led The LED to print.
     */
    static void printLed(const LED * led);

    /**
     * Switch on all LEDs use given brightness in all colors.
     * 
     * @param brightness The brightness to set.
     */
    static void allOn(const Brightness brightness);

    /**
     * Switch off all LEDs.
     */
    static void allOff();

    /**
     * Retrieve the LED on given index.
     */
    static LED * get(int index);

    /**
     * Shift a layer of the cube to SPI for the given BAM tick.
     * 
     * @param index index of the layer
     * @param tick current BAM tick
     */
    static void shiftLayerForTick(const int index, const int tick);

private:
    static uint8_t rows;
    static uint8_t cols;
    static uint8_t layers;
    static LinkedList<LED *> cubeLed;

    /**
     * Prepares the LEDs for the byte wise SPI transfer.
     * 
     * @param index from 0 .. getLayerSize()-1
     * @param tick current BAM tick (4-Bit BAM has 16 ticks)
     * @param shift current shift position
     */
    static void transferLEDs(const int index, const int tick, int *shift);

    /**
     * Shift one color pin setting of a LED into the transfer byte.
     * 
     * @param tick current BAM tick
     * @param brightness the brightness setting for the color
     * @param shift current shift position
     * @param transferByte the transfer byte to prepare
     */
    static void shiftAndTransferLedColor(const int tick, const Brightness *brightness, int *shift, uint8_t *transferByte);

    /**
     * Transfer the prepared transfer byte to SPI and prepare for the next transfer byte.
     * 
     * @param shift current shift position
     * @param transferByte current state of the transfer byte
     */
    static void shiftToSPI(int *shift, uint8_t *transferByte);
};
extern CubeClass Cube;

#endif