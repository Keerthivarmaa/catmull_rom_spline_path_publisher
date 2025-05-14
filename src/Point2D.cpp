#include "spline_path_generator/Point2D.h"

// Operator Overloading for Vector-style Addition
Point2D Point2D::operator+(const Point2D& other) const {
    return Point2D(this->x + other.x, this->y + other.y);
}

// Operator Overloading for Scalar Multiplication
Point2D Point2D::operator*(double scalar) const {
    return Point2D(this->x * scalar, this->y * scalar);
}

