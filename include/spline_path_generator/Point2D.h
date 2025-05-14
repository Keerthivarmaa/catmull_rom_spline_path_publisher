#ifndef POINT2D_H
#define POINT2D_H

struct Point2D {
    double x, y;

    // Constructor
    Point2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    // Operator Overloading
    Point2D operator+(const Point2D& other) const;
    Point2D operator*(double scalar) const;
};

#endif  // POINT2D_H

