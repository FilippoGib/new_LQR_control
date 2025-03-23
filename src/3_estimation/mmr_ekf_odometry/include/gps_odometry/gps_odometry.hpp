#include <mmr_odometry/mmr_odometry.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

using namespace GeographicLib;


class GPSOdometry : public MmrOdometry
{
private:
    /* Initial vehicle yaw */
    double initial_yaw;
    /* Initial vehicle UTM position */
    Vector2d initial_utm_position;
    /* Translation matrix for converting GPS localization info to local frame */
    Matrix3d mat_T;
    /* Local cartesian frame */
    GeographicLib::LocalCartesian *proj;
    /* Yaw difference between global and local frame. THIS IS EXTREMELY IMPORTANT since it's the magic number that makes GPS ROS compatible */
    double yaw_offset;

public:
    GPSOdometry(Vector2d initial_lat_lon_position);
    Vector3d getGlobalOdometry(const Vector2d actual_lat_lon_position);
    Vector2d getLocalOdometry(const Vector3d actual_x_y_global_position);

    void setYawOffset(const double yaw_offset);

};