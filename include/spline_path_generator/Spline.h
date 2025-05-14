#ifndef SPLINE_H
#define SPLINE_H

#include "spline_path_generator/Point2D.h"
#include <vector>

class Spline {
public:
    // Constructor
    Spline(const std::vector<Point2D>& control_points, double alpha = 0.5, int resolution = 10);

    // Computes the Centripetal Catmull-Rom Spline
    std::vector<Point2D> computeCentripetalCatmullRomSpline() const;

private:
    // Helper function for alpha-distance
    double getAlphaDist(const Point2D& p0, const Point2D& p1) const;

    std::vector<Point2D> control_points_;
    double alpha_;
    int resolution_;
};

#endif  // SPLINE_H

