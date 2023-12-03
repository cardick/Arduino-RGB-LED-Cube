#ifndef Vector3D_h
#define Vector3D_h

#include <stdint.h>

/**
 * Direction flags for the base directions.
 */
enum Direction
{
  Zero = 0 << 0,
  Front = 1 << 0,
  Left = 1 << 1,
  Up = 1 << 2,
  Back = 1 << 3,
  Right = 1 << 4,
  Down = 1 << 5
};

/**
 * Structure for a vector in a cubes or cuboid space.
 */
typedef struct
{
  int x;
  int y;
  int z;
} Vector3D;

/**
 * Structure for point in a cube or cuboid space.
 */
typedef struct
{
  uint8_t x;
  uint8_t y;
  uint8_t z;
} Point3D;

/**
 * Vector class defines statis methods to work with vectors.
 */
class VectorClass
{
public:
  /**
   * Set direction coordinates to get the unit vector of the direction. Can be
   * used to set a direction by using the Direction enum (e.g. Left|Up, Right,
   * Down|Back, Right|Up|Front).
   *
   * @param vector - the vector to set the coordinates
   * @param direction - the direction to set
   */
  static void setDirection(Vector3D *vector, const Direction direction);

  /**
   * Set the direction coordinates to get the unit vector of the direction.
   * Could be used to get a random direction.
   *
   * @param vector - the vector to set the coordinates
   * @param direction - unless there are 6 base directions the max value is
   *    maximum value of 6 bits (63)
   */
  static void setDirection(Vector3D *vector, const uint8_t direction);
  /**
   * Add vector a to vector b, where vector a holds the product of both.
   */
  static void add(Vector3D *vectorA, const Vector3D *vectorB);

  /**
   * Add vector a to vector b and stores the result in the product vector.
   */
  static void add(Vector3D *product, const Vector3D *vectorA, const Vector3D *vectorB);

  /**
   * Scalar addition to a vector
   */
  static void add(Vector3D *vector, int scalar);

  /**
   * Scalar multiplication to a vector.
   */
  static void multiply(Vector3D *vector, int scalar);

  /**
   * Building the magnitude of a vector.
   */
  static double magnitude(const Vector3D *vector);

  /**
   * Sets the direction of the vector to its opposite.
   *
   * @param vector the vector to change
   */
  static void inverse(Vector3D *vector);

  /**
   * Build the cross product of two vectors. The cross product is used to evaluate
   * a perpendicular vector.
   *
   * @param cross the resulting cross product
   * @param vectorA
   * @param vectorB
   */
  static void crossProduct(Vector3D *cross, const Vector3D *vectorA, const Vector3D *vectorB);

  /**
   * Determines the dot product of vector a and vector b. The dot product is 0 when
   * the vectors are perpendicular to each other.
   */
  static int getDotProduct(const Vector3D *vectorA, const Vector3D *vectorB);

  /**
   * Indicats whether the given vector is the zero vector (0,0,0).
   */
  static bool isZeroVector(const Vector3D *vector);
  /**
   * Checks whether vector a is equal to vector b
   */
  static bool equals(const Vector3D *vectorA, const Vector3D *vectorB);

  /**
   * Print a vector with a text.
   */
  static void print(const char * text, const Vector3D *vector);

  /**
   * Print a coordinates of a point with a text.
   */
  static void print(const char * text, const Point3D *point);
};
extern VectorClass Vector;

#endif