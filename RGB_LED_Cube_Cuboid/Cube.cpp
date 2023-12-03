#include <SPI.h>
#include "LedCube.h"
CubeClass Cube;

uint8_t CubeClass::rows;
uint8_t CubeClass::cols;
uint8_t CubeClass::layers;
LinkedList<LED *> CubeClass::cubeLed;

void CubeClass::init(uint8_t rows, uint8_t cols, uint8_t layers)
{
    CubeClass::rows = rows;
    CubeClass::cols = cols;
    CubeClass::layers = layers;
    cubeLed = LinkedList<LED *>();

    for (int k = 0; k < layers; k++)
    {
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                // init the led
                LED *elem = new LED();

                // init the color
                Color *c = new Color();
                c->red = Off;
                c->green = Off;
                c->blue = Off;
                // init position
                Point3D *p = new Point3D();
                p->x = i;
                p->y = j;
                p->z = k;

                elem->color = c;
                elem->position = p;

                // add led to cube
                cubeLed.add(elem);
            }
        }
    }
}

uint8_t CubeClass::getRowSize()
{
    return rows;
}

uint8_t CubeClass::getColumnSize()
{
    return cols;
}

uint8_t CubeClass::getLayerSize()
{
    return layers;
}

/**
 * Set a point to the corner coordinates defined by the direction.
 *
 * @param point The point that should represent the corner.
 * @param direction The direction vector that points to the corner.
 */
void CubeClass::setCorner(Point3D *point, const Vector3D *direction)
{
    point->x = direction->x <= 0 ? 0 : rows - 1;
    point->y = direction->y <= 0 ? 0 : cols - 1;
    point->z = direction->z <= 0 ? 0 : layers - 1;
}

void CubeClass::print()
{
    Serial.print("Cube( {rows: ");
    Serial.print(rows);
    Serial.print(", cols: ");
    Serial.print(cols);
    Serial.print(", layers: ");
    Serial.print(layers);
    Serial.print(" , ledAmount: ");
    Serial.print(cubeLed.size());
    Serial.println(" })");
}

void CubeClass::printJsonLeds()
{
    Serial.print("leds: [");
    for (int i = 0; i < cubeLed.size(); i++)
    {
        LED *elem = cubeLed.get(i);
        if (i == 0)
        {
            Serial.print(" {");
        }
        Serial.print(" position: {");
        Serial.print(elem->position->x);
        Serial.print(", ");
        Serial.print(elem->position->y);
        Serial.print(", ");
        Serial.print(elem->position->z);
        Serial.print("}, color: {");
        Serial.print(elem->color->red);
        Serial.print(", ");
        Serial.print(elem->color->green);
        Serial.print(", ");
        Serial.print(elem->color->blue);
        if (i < cubeLed.size() - 1)
        {
            Serial.print("} ");
            Serial.println("},");
            Serial.print("\t{");
        }
        else
        {
            Serial.print("} ");
            Serial.print("}");
        }
    }
    Serial.println("]");
}

void CubeClass::printLed(const LED *led)
{
    Serial.print("LED {");
    Serial.print(" position: {");
    Serial.print(led->position->x);
    Serial.print(", ");
    Serial.print(led->position->y);
    Serial.print(", ");
    Serial.print(led->position->z);
    Serial.print("}, color: {");
    Serial.print(led->color->red);
    Serial.print(", ");
    Serial.print(led->color->green);
    Serial.print(", ");
    Serial.print(led->color->blue);
    Serial.println("} }");
}

/**
 * Switch on all LEDs in white with given brightness.
 */
void CubeClass::allOn(const Brightness brightness)
{
    for (int i = 0; i < cubeLed.size(); i++)
    {
        LED *elem = cubeLed.get(i);
        elem->color->red = brightness;
        elem->color->green = brightness;
        elem->color->blue = brightness;
    }
}
/**
 * Switch off all LEDs.
 */
void CubeClass::allOff()
{
    for (int i = 0; i < cubeLed.size(); i++)
    {
        LED *elem = cubeLed.get(i);
        elem->color->red = Off;
        elem->color->green = Off;
        elem->color->blue = Off;
    }
}

LED *CubeClass::get(int index)
{
    return cubeLed.get(index);
}

void CubeClass::shiftLayerForTick(const int index, const int tick)
{
    // calculate the shift offset to begin shifting out the layer
    // there is maybe an offset of unused ports on the shift registers; this depends on the cubes size
    int shift = (((getRowSize() * 3 * getColumnSize()) + getLayerSize()) % 8) - 1;

    // correct modulo 8 is 0
    shift = shift == -1 ? 7 : shift;

    // transfer LED settings to SPI
    transferLEDs(index, tick, &shift);
}

/**
 * This is doing the shift out and the spy transfer
 * @param index from 0 .. getLayerSize()-1
 * @param tick current BAM tick (4-Bit BAM has 16 ticks)
 * @param shift current shift position
 */
void CubeClass::transferLEDs(const int index, const int tick, int *shift)
{
    uint8_t transferByte = 0b00000000;

    // ensure the layer index is between 0 (bottom) and layers-1 (top)
    // evaluate the index of the last led in current (start) and previous layer (end).
    int startIndex = (((index % getLayerSize()) + 1) * ((getRowSize() * getColumnSize())) - 1);
    int endIndex = startIndex - (getRowSize() * getColumnSize());

    // The idea is to shift out the led information from back to front from the perspective of the
    // daisy chained shift registers. Shift out only the current layer for the current tick given by
    // the BAM duty cycle. The rest is done by Arduino by repetetive calls of this method.

    // now we have to shift out each LED from highest to lowest in the order blue, green, red
    for (int i = startIndex; i > endIndex; i--)
    {
        LED *current = get(i);
        shiftAndTransferLedColor(tick, &current->color->blue, shift, &transferByte);
        shiftAndTransferLedColor(tick, &current->color->green, shift, &transferByte);
        shiftAndTransferLedColor(tick, &current->color->red, shift, &transferByte);
    }

    // Shift out the layer anodes, the logic bases on the idea that the anodes are at the
    // beginning of the daisy chained shift registers and that the bottem layer is
    // connected in first position (MSBFIRST)
    for (int i = getLayerSize() - 1; i >= 0; i--)
    {
        if (i == index)
        {
            transferByte = transferByte | (1 << *shift);
        }
        *shift = *shift - 1;
    }

    shiftToSPI(shift, &transferByte);
}

void CubeClass::shiftAndTransferLedColor(const int tick, const Brightness *brightness, int *shift, uint8_t *transferByte)
{
    // When tick contains the brightness then the led color pin is on;
    // otherwise it is off. This ensures the 4 bit BAM.
    if (((uint8_t)*brightness) - (1 << tick) >= 0)
    {
        *transferByte = *transferByte | (1 << *shift);
    }
    else
    {
        *transferByte = *transferByte | (0 << *shift);
    }
    *shift = *shift - 1;

    shiftToSPI(shift, transferByte);
}

void CubeClass::shiftToSPI(int *shift, uint8_t *transferByte)
{
    // is byte ready to transfer
    if (*shift < 0)
    {
        SPI.transfer(*transferByte);

        // prepare for next byte to shift
        *transferByte = 0b00000000;
        *shift = 7;
    }
}