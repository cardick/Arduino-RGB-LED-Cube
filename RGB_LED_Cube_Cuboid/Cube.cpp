#include "LedCube.h"
CubeClass Cube;

uint8_t CubeClass::rows;
uint8_t CubeClass::cols;
uint8_t CubeClass::layers;
LinkedList<LED*> CubeClass::cubeLed;

void CubeClass::init(uint8_t rows, uint8_t cols, uint8_t layers)
{
    CubeClass::rows = rows;
    CubeClass::cols = cols;
    CubeClass::layers = layers;
    cubeLed = LinkedList<LED*>();

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            for (int k = 0; k < layers; k++)
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
                p->x=i;
                p->y=j;
                p->z=k;

                elem->color=c;
                elem->position=p;
                
                // add led to cube
                cubeLed.add(elem);
            }
        }
    }
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

void CubeClass::print() {
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

void CubeClass::printJsonLeds() {
    Serial.print("leds: [");
    for (int i = 0; i < cubeLed.size(); i++)
    {
        LED *elem = cubeLed.get(i);
        if(i==0) {
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
        if(i < cubeLed.size()-1) {
            Serial.print("} ");
            Serial.println("},");
            Serial.print("\t{");
        } else {
            Serial.print("} ");
            Serial.print("}");
        }
    }
    Serial.println("]");
}