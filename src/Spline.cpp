#include "spline_path_generator/Spline.h"
#include <cmath>

Spline::Spline(const std::vector<Point2D>& control_points, double alpha, int resolution)
    : control_points_(control_points), alpha_(alpha), resolution_(resolution) {}

double Spline::getAlphaDist(const Point2D& p0, const Point2D& p1) const {
    return std::pow(std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2)), alpha_);
}
namespace {
Point2D interpolate(const Point2D& p0, const Point2D& p1, double t0, double t1, double t) {
  double factor = (t - t0) / (t1 - t0);
  return {
    (1 - factor) * p0.x + factor * p1.x,
    (1 - factor) * p0.y + factor * p1.y
  };
}
}
std::vector<Point2D> Spline::computeCentripetalCatmullRomSpline() const {
    std::vector<Point2D> result;
    if (control_points_.size() < 4) {
        return result;  // Need at least 4 points
    }

    for (size_t i = 1; i + 2 < control_points_.size(); ++i) {
        Point2D p0 = control_points_[i - 1];
        Point2D p1 = control_points_[i];
        Point2D p2 = control_points_[i + 1];
        Point2D p3 = control_points_[i + 2];

        double t0 = 0.0;
        double t1 = t0 + getAlphaDist(p0, p1);
        double t2 = t1 + getAlphaDist(p1, p2);
        double t3 = t2 + getAlphaDist(p2, p3);

        for (int j = 0; j <= resolution_; ++j) {
            double t = t1 + j * (t2 - t1) / resolution_;

            Point2D A1 = interpolate(p0, p1, t0, t1, t);
            Point2D A2 = interpolate(p1, p2, t1, t2, t);
            Point2D A3 = interpolate(p2, p3, t2, t3, t);

            Point2D B1 = interpolate(A1, A2, t0, t2, t);
            Point2D B2 = interpolate(A2, A3, t1, t3, t);

            Point2D C = interpolate(B1, B2, t1, t2, t);
            result.push_back(C);
        }
    }

    return result;
}

