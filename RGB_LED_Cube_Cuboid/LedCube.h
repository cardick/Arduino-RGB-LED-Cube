
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
  Off = 0 << 0,
  Low = 1 << 0,
  Medium = 1 << 1,
  High = 1 << 2,
  Full = 1 << 3,
};

typedef struct {
    Brightness red;
    Brightness green;
    Brightness blue;
} Color;

struct LED {
    Point3D *position;
    Color *color;
};

typedef struct LED LED;

class CubeClass {
    public:
    /**
     * Initialize the cube dimension.
     */
    static void init(uint8_t rows, uint8_t cols, uint8_t layers);

    /**
     * Set the point to the corner coordinates of the cube in given direction.
     */
    static void setCorner(Point3D * point, const Vector3D * direction);

    /**
     * Print short summary of the cubes state. 
     */
    static void print();

    /**
     * Print cubes state of the LEDs in JSON format.
    */
    static void printJsonLeds();

    private:
    static uint8_t rows;
    static uint8_t cols;
    static uint8_t layers;
    static LinkedList<LED*> cubeLed;
};
extern CubeClass Cube;

#endif