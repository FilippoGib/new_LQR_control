#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include <optional>
#include <eigen3/Eigen/Geometry>
#include <boost/math/interpolators/pchip.hpp>

namespace lqr {

class SplinePath {
public:
    // Empty default constructor
    SplinePath() = default;

    // Construct from raw, non-uniformly spaced points (Eigen::Vector2f)
    SplinePath(const std::vector<Eigen::Vector2f>& pts) {
        const size_t N = pts.size();
        s_.resize(N);
        s_[0] = 0.0;
        for (size_t i = 1; i < N; ++i) {
            double dx = static_cast<double>(pts[i].x() - pts[i-1].x());
            double dy = static_cast<double>(pts[i].y() - pts[i-1].y());
            s_[i] = s_[i-1] + std::hypot(dx, dy);
        }

        // Extract x and y into separate double vectors
        std::vector<double> xs(N), ys(N);
        for (size_t i = 0; i < N; ++i) {
            xs[i] = static_cast<double>(pts[i].x());
            ys[i] = static_cast<double>(pts[i].y());
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
    Eigen::Vector2f evaluate(double u) const {
        float x = static_cast<float>((*spline_x_)(u));
        float y = static_cast<float>((*spline_y_)(u));
        return Eigen::Vector2f(x, y);
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
    std::vector<double> s_;  // chord-length parameters
    std::optional<boost::math::interpolators::pchip<std::vector<double>>> spline_x_;
    std::optional<boost::math::interpolators::pchip<std::vector<double>>> spline_y_;
};

} // namespace lqr
