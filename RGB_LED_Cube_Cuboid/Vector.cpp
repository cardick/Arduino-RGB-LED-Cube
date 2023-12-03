#include <Arduino.h>
#include "Vector3D.h"

/**
 * Set direction coordinates to get the unit vector of the direction. Can be
 * used to set a direction by using the Direction enum (e.g. Left|Up, Right,
 * Down|Back, Right|Up|Front).
 *
 * @param vector - the vector to set the coordinates
 * @param direction - the direction to set
 */
void VectorClass::setDirection(Vector3D *vector, const Direction direction)
{
    setDirection(vector, (uint8_t)direction);
}

/**
 * Set the direction coordinates to get the unit vector of the direction.
 * Could be used to get a random direction.
 *
 * @param vector - the vector to set the coordinates
 * @param direction - unless there are 6 base directions the max value is
 *    maximum value of 6 bits (63)
 */
void VectorClass::setDirection(Vector3D *vector, const uint8_t direction)
{
    // direction = constrain(direction, 0, 63);

    // zero vector - no direction
    vector->x = 0;
    vector->y = 0;
    vector->z = 0;

    if(direction>63) {
        return;
    }

    // set directions to vector
    for (uint8_t i = 0; i < 6; i++)
    {
        switch (direction & (1 << i))
        {
        case Front:
            vector->x += 1;
            break;
        case Left:
            vector->y += 1;
            break;
        case Up:
            vector->z += 1;
            break;
        case Back:
            vector->x -= 1;
            break;
        case Right:
            vector->y -= 1;
            break;
        case Down:
            vector->z -= 1;
            break;
        }
    }
}

/**
 * Add vector a to vector b, where vector a holds the product of both.
 */
void VectorClass::add(Vector3D *vectorA, const Vector3D *vectorB)
{
    vectorA->x += vectorB->x;
    vectorA->y += vectorB->y;
    vectorA->z += vectorB->z;
}

/**
 * Add vector a to vector b and stores the result in the product vector.
 */
void VectorClass::add(Vector3D *product, const Vector3D *vectorA, const Vector3D *vectorB)
{
    product->x = vectorA->x + vectorB->x;
    product->y = vectorA->y + vectorB->y;
    product->z = vectorA->z + vectorB->z;
}

/**
 * Scalar addition to a vector
 */
void VectorClass::add(Vector3D *vector, int scalar)
{
    vector->x += scalar;
    vector->y += scalar;
    vector->z += scalar;
}

/**
 * Scalar multiplication to a vector
 */
void VectorClass::multiply(Vector3D *vector, int scalar)
{
    vector->x *= scalar;
    vector->y *= scalar;
    vector->z *= scalar;
}

double VectorClass::magnitude(const Vector3D *vector)
{
    return sqrt(sq(vector->x) + sq(vector->y) + sq(vector->z));
}

/**
 * Sets the direction of the vector to its opposite.
 *
 * @param vector the vector to change
 */
void VectorClass::inverse(Vector3D *vector)
{
    vector->x = vector->x * (-1);
    vector->y = vector->y * (-1);
    vector->z = vector->z * (-1);
}

/**
 * Build the cross product of two vectors. The cross product is used to evaluate
 * a perpendicular vector.
 *
 * @param cross the resulting cross product
 * @param vectorA
 * @param vectorB
 */
void VectorClass::crossProduct(Vector3D *cross, const Vector3D *vectorA, const Vector3D *vectorB)
{
    cross->x = (vectorA->y * vectorB->z) - (vectorA->z - vectorB->y);
    cross->y = (vectorA->z * vectorB->x) - (vectorA->x - vectorB->z);
    cross->z = (vectorA->x * vectorB->y) - (vectorA->y * vectorB->x);
}

/**
 * Determines the dot product of vector a and vector b. The dot product is 0 when
 * the vectors are perpendicular to each other.
 */
int VectorClass::getDotProduct(const Vector3D *vectorA, const Vector3D *vectorB)
{
    return (vectorA->x * vectorB->x) + (vectorA->y * vectorB->y) + (vectorA->z * vectorB->z);
}

/**
 * Indicats whether the given vector is the zero vector (0,0,0).
 */
bool VectorClass::isZeroVector(const Vector3D *vec)
{
    return vec->x == 0 && vec->y == 0 && vec->z == 0;
}

/**
 * Checks whether vector a is equal to vector b
 */
bool VectorClass::equals(const Vector3D *vectorA, const Vector3D *vectorB)
{
    return (vectorA->x == vectorB->x) && (vectorA->y == vectorB->y) && (vectorA->z == vectorB->z);
}

/**
 * Print to Serial out.
 */
void VectorClass::print(const char * text, const Vector3D *vector)
{
    Serial.print(text);
    Serial.print("(");
    Serial.print(vector->x);
    Serial.print(",");
    Serial.print(vector->y);
    Serial.print(",");
    Serial.print(vector->z);
    Serial.println(")");
}

/**
 * Print to Serial out.
 */
void VectorClass::print(const char * text, const Point3D *point)
{
    Serial.print(text);
    Serial.print("(");
    Serial.print(point->x);
    Serial.print(",");
    Serial.print(point->y);
    Serial.print(",");
    Serial.print(point->z);
    Serial.println(")");
}