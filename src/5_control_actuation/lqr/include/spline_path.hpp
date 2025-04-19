#pragma once

#include <vector>
#include <cmath>
#include <math.h>
#include <limits>
#include <optional>
#include <boost/math/interpolators/pchip.hpp>

namespace lqr{
struct Point { double x, y; };

class SplinePath {
public:
    // Empty default constructor
    SplinePath() = default;
    // Construct from raw, non-uniformly spaced points
    SplinePath(const std::vector<Point>& pts) {
        const size_t N = pts.size();
        s_.resize(N);
        s_[0] = 0.0;
        for (size_t i = 1; i < N; ++i) {
            double dx = pts[i].x - pts[i-1].x;
            double dy = pts[i].y - pts[i-1].y;
            s_[i] = s_[i-1] + std::hypot(dx, dy);
        }

        // Extract x and y into separate vectors
        std::vector<double> xs(N), ys(N);
        for (size_t i = 0; i < N; ++i) {
            xs[i] = pts[i].x;
            ys[i] = pts[i].y;
        }

        double nan = std::numeric_limits<double>::quiet_NaN();
        {
            auto s_copy = s_;
            spline_x_.emplace(std::move(s_copy), std::move(xs), nan, nan);
        }
        {
            auto s_copy = s_;
            spline_y_.emplace(std::move(s_copy), std::move(ys), nan, nan);
        }
    }

    /// Evaluate (x,y) at parameter u
    Point evaluate(double u) const {
        return { (*spline_x_)(u), (*spline_y_)(u) };
    }

    /// Compute yaw = atan2(dy/du, dx/du)
    double yaw(double u) const {
        double dx = spline_x_->prime(u);
        double dy = spline_y_->prime(u);
        return std::atan2(dy, dx);
    }

    /// Access raw chord-length parameter vector
    const std::vector<double>& params() const { return s_; }

private:
    std::vector<double> s_;
    std::optional<boost::math::interpolators::pchip<std::vector<double>>> spline_x_;
    std::optional<boost::math::interpolators::pchip<std::vector<double>>> spline_y_;
};

}