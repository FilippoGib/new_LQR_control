#include <mmr_odometry/mmr_odometry.hpp>

class IMUOdometry : public MmrOdometry
{
public:
    IMUOdometry();
    double getOdometry(const double actual_yaw, const Vector3d actual_accel_data);
};