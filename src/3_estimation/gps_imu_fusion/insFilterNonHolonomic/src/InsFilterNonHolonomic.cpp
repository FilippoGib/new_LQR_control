#include <InsFilterNonHolonomic.hpp>

InsFilterNonHolonomic::InsFilterNonHolonomic()
{
    this->filter_state = Matrix<double, 16, 1>();

    /* create initial error covariance */
    this->state_covariance = Matrix<double, 16, 16>::Identity() * 1e-9;
}

void InsFilterNonHolonomic::loadParameters(Vector3d gyroscope_noise,
    Vector3d gyroscope_bias_noise,
    Vector3d accelerometer_noise,
    Vector3d accelerometer_bias_noise,
    double r_vel = 0.0f,
    double r_pos = 0.005f,
    double gyroscope_bias_decay_factor = 1.0f,
    double accel_bias_decay_factor = 1.0f,
    double zero_velocity_constraint_noise = 1e-3
)
{
    this->gyroscope_noise = gyroscope_noise;
    this->gyroscope_bias_noise = gyroscope_bias_noise;
    this->accelerometer_noise = accelerometer_noise;
    this->accelerometer_bias_noise = accelerometer_bias_noise;
    this->r_vel = r_vel;
    this->r_pos = r_pos;
    this->gyroscope_bias_decay_factor = gyroscope_bias_decay_factor;
    this->accel_bias_decay_factor = accel_bias_decay_factor;
    this->zero_velocity_constraint_noise = zero_velocity_constraint_noise;
}

void InsFilterNonHolonomic::setInitState(Vector4d q_init, 
    Vector3d gyro_bias_init, 
    Vector3d position_init, 
    Vector3d velocity_init, 
    Vector3d accel_bias_init)
{
    /* Set initial Quaternion */
    this->filter_state(0, 0) = q_init(0);   /* qx */
    this->filter_state(1, 0) = q_init(1);   /* qy */
    this->filter_state(2, 0) = q_init(2);   /* qz */
    this->filter_state(3, 0) = q_init(3);   /* qw */

    this->filter_state(4, 0) = gyro_bias_init(0);
    this->filter_state(5, 0) = gyro_bias_init(1);
    this->filter_state(6, 0) = gyro_bias_init(2);

    this->filter_state(7, 0) = position_init(0);
    this->filter_state(8, 0) = position_init(1);
    this->filter_state(9, 0) = position_init(2);

    this->filter_state(10, 0) = velocity_init(0);
    this->filter_state(11, 0) = velocity_init(1);
    this->filter_state(12, 0) = velocity_init(2);

    this->filter_state(13, 0) = accel_bias_init(0);
    this->filter_state(14, 0) = accel_bias_init(1);
    this->filter_state(15, 0) = accel_bias_init(2);
}

void InsFilterNonHolonomic::setRefLocation(double latitude, double longitude, double altitude)
{
    this->reference_location.latitude = latitude;
    this->reference_location.longitude = longitude;
    this->reference_location.altitude = altitude;
}

void InsFilterNonHolonomic::setState(Vector3d accel_data, Vector3d gyro_data)
{
    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    double gbX = this->filter_state(4, 0);
    double gbY = this->filter_state(5, 0);
    double gbZ = this->filter_state(6, 0);
    double pn = this->filter_state(7, 0);
    double pe = this->filter_state(8, 0);
    double pd = this->filter_state(9, 0);
    double vn = this->filter_state(10, 0);
    double ve = this->filter_state(11, 0);
    double vd = this->filter_state(12, 0);
    double abX = this->filter_state(13, 0);
    double abY = this->filter_state(14, 0);
    double abZ = this->filter_state(15, 0);

    /* Retrieve sensors data */
    double amX = accel_data(0);
    double amY = accel_data(1);
    double amZ = accel_data(2);
    
    double gmX = gyro_data(0);
    double gmY = gyro_data(1);
    double gmZ = gyro_data(2);

    double lambda_gyro = 1 - this->gyroscope_bias_decay_factor;
    double lambda_accel = 1 - this->accel_bias_decay_factor;

    /* NED system -> Gravity is on the Z axis */
    double gravX = 0.0f;
    double gravY = 0.0f;
    double gravZ = -9.81f;

    /* Set new system state. To understand the following code read insFilterNonholonomic documentation */
    
    /* New predicted orientation */
    this->filter_state(0, 0) = q0 + dt*q1*(gbX/2 - gmX/2) + dt*q2*(gbY/2 - gmY/2) + dt*q3*(gbZ/2 - gmZ/2);  //q0
    this->filter_state(1, 0) = q1 - dt*q0*(gbX/2 - gmX/2) + dt*q3*(gbY/2 - gmY/2) - dt*q2*(gbZ/2 - gmZ/2);  //q1
    this->filter_state(2, 0) = q2 - dt*q3*(gbX/2 - gmX/2) - dt*q0*(gbY/2 - gmY/2) + dt*q1*(gbZ/2 - gmZ/2);  //q2
    this->filter_state(3, 0) = q3 + dt*q2*(gbX/2 - gmX/2) - dt*q1*(gbY/2 - gmY/2) - dt*q0*(gbZ/2 - gmZ/2);  //q3
    
    /* New predicted gyro_bias */
    this->filter_state(4, 0) = -gbX*(dt*lambda_gyro - 1);  //gyro_bias_x
    this->filter_state(5, 0) = -gbY*(dt*lambda_gyro - 1);  //gyro_bias_y
    this->filter_state(6, 0) = -gbZ*(dt*lambda_gyro - 1);  //gyro_bias_z

    /* New predicted position */
    this->filter_state(7, 0) = pn + dt*vn;  //pos_N
    this->filter_state(8, 0) = pe + dt*ve;  //pos_E
    this->filter_state(9, 0) = pd + dt*vd;  //pos_D

    /* New predicted velocies */
    this->filter_state(10, 0) = vn + dt*(q0*(q0*(abX - amX) - q3*(abY - amY) + q2*(abZ - amZ)) - gravX + q2*(q1*(abY - amY) - q2*(abX - amX) + q0*(abZ - amZ)) + q1*(q1*(abX - amX) + q2*(abY - amY) + q3*(abZ - amZ)) - q3*(q3*(abX - amX) + q0*(abY - amY) - q1*(abZ - amZ)));  //vel_N
    this->filter_state(11, 0) = ve + dt*(q0*(q3*(abX - amX) + q0*(abY - amY) - q1*(abZ - amZ)) - gravY - q1*(q1*(abY - amY) - q2*(abX - amX) + q0*(abZ - amZ)) + q2*(q1*(abX - amX) + q2*(abY - amY) + q3*(abZ - amZ)) + q3*(q0*(abX - amX) - q3*(abY - amY) + q2*(abZ - amZ)));  //vel_E
    this->filter_state(12, 0) = vd + dt*(q0*(q1*(abY - amY) - q2*(abX - amX) + q0*(abZ - amZ)) - gravZ + q1*(q3*(abX - amX) + q0*(abY - amY) - q1*(abZ - amZ)) - q2*(q0*(abX - amX) - q3*(abY - amY) + q2*(abZ - amZ)) + q3*(q1*(abX - amX) + q2*(abY - amY) + q3*(abZ - amZ)));

    /* New predicted accel_bias */
    this->filter_state(13, 0) = -abX*(dt*lambda_accel - 1);  //accel_bias_x
    this->filter_state(14, 0) = -abY*(dt*lambda_accel - 1);  //accel_bias_y
    this->filter_state(15, 0) = -abZ*(dt*lambda_accel - 1);  //accel_bias_z
    // InsFilterNonHolonomicTypes::Quaternion::repair({
    //     this->filter_state(0, 0),
    //     this->filter_state(1, 0),
    //     this->filter_state(2, 0),
    //     this->filter_state(3, 0)
    // });
    double n = sqrt(pow(this->filter_state(0, 0), 2) + pow(this->filter_state(1, 0), 2) + pow(this->filter_state(2, 0), 2) + pow(this->filter_state(3, 0), 2));
    this->filter_state(0, 0) /= n;
    this->filter_state(1, 0) /= n; 
    this->filter_state(2, 0) /= n;
    this->filter_state(3, 0) /= n;

    if (this->filter_state(0, 0) < 0)
    {
        this->filter_state(0, 0) *= -1;
        this->filter_state(1, 0) *= -1; 
        this->filter_state(2, 0) *= -1;
        this->filter_state(3, 0) *= -1;
    }
}

MatrixXd InsFilterNonHolonomic::stateTransitionJacobianFcn(Vector3d accel_data, Vector3d gyro_data)
{
    MatrixXd F = Matrix<double, 16, 16>();

    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    double gbX = this->filter_state(4, 0);
    double gbY = this->filter_state(5, 0);
    double gbZ = this->filter_state(6, 0);
    // double pn = this->filter_state(7, 0);
    // double pe = this->filter_state(8, 0);
    // double pd = this->filter_state(9, 0);
    // double vn = this->filter_state(10, 0);
    // double ve = this->filter_state(11, 0);
    // double vd = this->filter_state(12, 0);
    double abX = this->filter_state(13, 0);
    double abY = this->filter_state(14, 0);
    double abZ = this->filter_state(15, 0);

    /* Retrieve sensors data */
    double amX = accel_data(0);
    double amY = accel_data(1);
    double amZ = accel_data(2);
    double gmX = gyro_data(0);
    double gmY = gyro_data(1);
    double gmZ = gyro_data(2);

    double lambda_gyro = 1 - this->gyroscope_bias_decay_factor;
    double lambda_accel = 1 - this->accel_bias_decay_factor;

    F <<
        1,                                                            dt*(gbX/2 - gmX/2),                                           dt*(gbY/2 - gmY/2),                                           dt*(gbZ/2 - gmZ/2),         (dt*q1)/2,         (dt*q2)/2,         (dt*q3)/2, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        -dt*(gbX/2 - gmX/2),                                          1,                                          -dt*(gbZ/2 - gmZ/2),                                           dt*(gbY/2 - gmY/2),        -(dt*q0)/2,         (dt*q3)/2,        -(dt*q2)/2, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        -dt*(gbY/2 - gmY/2),                                           dt*(gbZ/2 - gmZ/2),                                                            1,                                          -dt*(gbX/2 - gmX/2),        -(dt*q3)/2,        -(dt*q0)/2,         (dt*q1)/2, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        -dt*(gbZ/2 - gmZ/2),                                          -dt*(gbY/2 - gmY/2),                                           dt*(gbX/2 - gmX/2),                                                            1,         (dt*q2)/2,        -(dt*q1)/2,        -(dt*q0)/2, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0, 1 - dt*lambda_gyro,                 0,                 0, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0, 1 - dt*lambda_gyro,                 0, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0, 1 - dt*lambda_gyro, 0, 0, 0,  0,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 1, 0, 0, dt,  0,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 1, 0,  0, dt,  0,                              0,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 0, 1,  0,  0, dt,                              0,                              0,                              0,
        dt*(2*q0*(abX - amX) - 2*q3*(abY - amY) + 2*q2*(abZ - amZ)),  dt*(2*q1*(abX - amX) + 2*q2*(abY - amY) + 2*q3*(abZ - amZ)),  dt*(2*q1*(abY - amY) - 2*q2*(abX - amX) + 2*q0*(abZ - amZ)), -dt*(2*q3*(abX - amX) + 2*q0*(abY - amY) - 2*q1*(abZ - amZ)),                 0,                 0,                 0, 0, 0, 0,  1,  0,  0, dt*(pow(q0,2) + pow(q1,2) - pow(q2,2) - pow(q3,2)),        -dt*(2*q0*q3 - 2*q1*q2),         dt*(2*q0*q2 + 2*q1*q3),
        dt*(2*q3*(abX - amX) + 2*q0*(abY - amY) - 2*q1*(abZ - amZ)), -dt*(2*q1*(abY - amY) - 2*q2*(abX - amX) + 2*q0*(abZ - amZ)),  dt*(2*q1*(abX - amX) + 2*q2*(abY - amY) + 2*q3*(abZ - amZ)),  dt*(2*q0*(abX - amX) - 2*q3*(abY - amY) + 2*q2*(abZ - amZ)),                 0,                 0,                 0, 0, 0, 0,  0,  1,  0,         dt*(2*q0*q3 + 2*q1*q2), dt*(pow(q0,2) - pow(q1,2) + pow(q2,2) - pow(q3,2)),        -dt*(2*q0*q1 - 2*q2*q3),
        dt*(2*q1*(abY - amY) - 2*q2*(abX - amX) + 2*q0*(abZ - amZ)),  dt*(2*q3*(abX - amX) + 2*q0*(abY - amY) - 2*q1*(abZ - amZ)), -dt*(2*q0*(abX - amX) - 2*q3*(abY - amY) + 2*q2*(abZ - amZ)),  dt*(2*q1*(abX - amX) + 2*q2*(abY - amY) + 2*q3*(abZ - amZ)),                 0,                 0,                 0, 0, 0, 0,  0,  0,  1,        -dt*(2*q0*q2 - 2*q1*q3),         dt*(2*q0*q1 + 2*q2*q3), dt*(pow(q0,2) - pow(q1,2) - pow(q2,2) + pow(q3,2)),
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 0, 0,  0,  0,  0,             1 - dt*lambda_accel,                              0,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 0, 0,  0,  0,  0,                              0,             1 - dt*lambda_accel,                              0,
        0,                                                            0,                                                            0,                                                            0,                 0,                 0,                 0, 0, 0, 0,  0,  0,  0,                              0,                              0,             1 - dt*lambda_accel;
    ;

    return F;
}

MatrixXd InsFilterNonHolonomic::processNoiseJacobianFcn()
{
    MatrixXd G = Matrix<double, 16, 12>();

    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    // double gbX = this->filter_state(4, 0);
    // double gbY = this->filter_state(5, 0);
    // double gbZ = this->filter_state(6, 0);
    // double pn = this->filter_state(7, 0);
    // double pe = this->filter_state(8, 0);
    // double pd = this->filter_state(9, 0);
    // double vn = this->filter_state(10, 0);
    // double ve = this->filter_state(11, 0);
    // double vd = this->filter_state(12, 0);
    // double abX = this->filter_state(13, 0);
    // double abY = this->filter_state(14, 0);
    // double abZ = this->filter_state(15, 0);

    G <<
        -(dt*q1)/2, -(dt*q2)/2, -(dt*q3)/2, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        (dt*q0)/2, -(dt*q3)/2,  (dt*q2)/2, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        (dt*q3)/2,  (dt*q0)/2, -(dt*q1)/2, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        -(dt*q2)/2,  (dt*q1)/2,  (dt*q0)/2, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0, -dt*(pow(q0,2) + pow(q1,2) - pow(q2,2) - pow(q3,2)),          dt*(2*q0*q3 - 2*q1*q2),         -dt*(2*q0*q2 + 2*q1*q3), 0, 0, 0,
        0,          0,          0, 0, 0, 0,         -dt*(2*q0*q3 + 2*q1*q2), -dt*(pow(q0,2) - pow(q1,2) + pow(q2,2) - pow(q3,2)),          dt*(2*q0*q1 - 2*q2*q3), 0, 0, 0,
        0,          0,          0, 0, 0, 0,          dt*(2*q0*q2 - 2*q1*q3),         -dt*(2*q0*q1 + 2*q2*q3), -dt*(pow(q0,2) - pow(q1,2) - pow(q2,2) + pow(q3,2)), 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0,
        0,          0,          0, 0, 0, 0,                               0,                               0,                               0, 0, 0, 0;
    ;

    return G;

}

MatrixXd InsFilterNonHolonomic::processNoiseCovariance()
{
    DiagonalMatrix<double, 3> gyro_var(this->gyroscope_noise(0), this->gyroscope_noise(1), this->gyroscope_noise(2));
    DiagonalMatrix<double, 3> gyro_bias_var(this->gyroscope_bias_noise(0), this->gyroscope_bias_noise(1), this->gyroscope_bias_noise(2));
    DiagonalMatrix<double, 3> accel_var(this->accelerometer_noise(0), this->accelerometer_noise(1), this->accelerometer_noise(2));
    DiagonalMatrix<double, 3> accel_bias_var(this->accelerometer_bias_noise(0),this->accelerometer_bias_noise(1), this->accelerometer_bias_noise(2));

    /* U is a 12x12 DIAGONAL matrix. It just contains the values of the previous diagonal matrices */
    MatrixXd U = Matrix<double, 12, 12>::Zero();

    U.diagonal()[0] = gyro_var.diagonal()[0];
    U.diagonal()[1] = gyro_var.diagonal()[1];
    U.diagonal()[2] = gyro_var.diagonal()[2];
    U.diagonal()[3] = gyro_bias_var.diagonal()[0];
    U.diagonal()[4] = gyro_bias_var.diagonal()[1];
    U.diagonal()[5] = gyro_bias_var.diagonal()[2];
    U.diagonal()[6] = accel_var.diagonal()[0];
    U.diagonal()[7] = accel_var.diagonal()[1];
    U.diagonal()[8] = accel_var.diagonal()[2];
    U.diagonal()[9] = accel_bias_var.diagonal()[0];
    U.diagonal()[10] = accel_bias_var.diagonal()[1];
    U.diagonal()[11] = accel_bias_var.diagonal()[2];

    return U;
}

void InsFilterNonHolonomic::setStateCovariance(MatrixXd F, MatrixXd U, MatrixXd G)
{
    /**
     * P = StateCovariance 
     * F = stateTransitionJacobianFcn
     * U = processNoiseCovariance
     * G = processNoiseJacobianFcn
    */
    MatrixXd Q = Matrix<double, 16, 16>();
    Q = G * U * G.transpose();
    this->state_covariance = F * this->state_covariance * F.transpose() + Q;
}

/* ***** PREDICT METHODS ***** */

MatrixXd InsFilterNonHolonomic::measurementFcnKinematics()
{
    MatrixXd h = Matrix<double, 2, 1>();

    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    // double gbX = this->filter_state(4, 0);
    // double gbY = this->filter_state(5, 0);
    // double gbZ = this->filter_state(6, 0);
    // double pn = this->filter_state(7, 0);
    // double pe = this->filter_state(8, 0);
    // double pd = this->filter_state(9, 0);
    double vn = this->filter_state(10, 0);
    double ve = this->filter_state(11, 0);
    double vd = this->filter_state(12, 0);
    // double abX = this->filter_state(13, 0);
    // double abY = this->filter_state(14, 0);
    // double abZ = this->filter_state(15, 0);

    h <<
        q0*(q1*vd + q0*ve - q3*vn) + q1*(q0*vd - q1*ve + q2*vn) + q2*(q3*vd + q2*ve + q1*vn) - q3*(q3*ve - q2*vd + q0*vn),
        q0*(q0*vd - q1*ve + q2*vn) - q1*(q1*vd + q0*ve - q3*vn) + q2*(q3*ve - q2*vd + q0*vn) + q3*(q3*vd + q2*ve + q1*vn)
    ;

    return h;
}

MatrixXd InsFilterNonHolonomic::measurementJacobianFcnKinematics()
{
    MatrixXd H = Matrix<double, 2, 16>();

    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    // double gbX = this->filter_state(4, 0);
    // double gbY = this->filter_state(5, 0);
    // double gbZ = this->filter_state(6, 0);
    // double pn = this->filter_state(7, 0);
    // double pe = this->filter_state(8, 0);
    // double pd = this->filter_state(9, 0);
    double vn = this->filter_state(10, 0);
    double ve = this->filter_state(11, 0);
    double vd = this->filter_state(12, 0);
    // double abX = this->filter_state(13, 0);
    // double abY = this->filter_state(14, 0);
    // double abZ = this->filter_state(15, 0);

    H <<
        2*q1*vd + 2*q0*ve - 2*q3*vn, 2*q0*vd - 2*q1*ve + 2*q2*vn, 2*q3*vd + 2*q2*ve + 2*q1*vn, 2*q2*vd - 2*q3*ve - 2*q0*vn, 0, 0, 0, 0, 0, 0, 2*q1*q2 - 2*q0*q3, pow(q0,2) - pow(q1,2) + pow(q2,2) - pow(q3,2),         2*q0*q1 + 2*q2*q3, 0, 0, 0,
        2*q0*vd - 2*q1*ve + 2*q2*vn, 2*q3*vn - 2*q0*ve - 2*q1*vd, 2*q3*ve - 2*q2*vd + 2*q0*vn, 2*q3*vd + 2*q2*ve + 2*q1*vn, 0, 0, 0, 0, 0, 0, 2*q0*q2 + 2*q1*q3,         2*q2*q3 - 2*q0*q1, pow(q0,2) - pow(q1,2) - pow(q2,2) + pow(q3,2), 0, 0, 0;
    ;

    return H;

}

MatrixXd InsFilterNonHolonomic:: measurementNoiseKinematics()
{
    MatrixXd R = Matrix<double, 2, 2>::Identity();

    R *= this->zero_velocity_constraint_noise;

    return R;
}


void InsFilterNonHolonomic::correctKinematics()
{

    MatrixXd h = this->measurementFcnKinematics();
    MatrixXd H = this->measurementJacobianFcnKinematics();
    MatrixXd R = this->measurementNoiseKinematics();
    Vector2d z = Vector2d::Zero();
    MatrixXd innovation_covariance = H * this->state_covariance * H.transpose() + R;
    MatrixXd innovation = z - h;
    MatrixXd W = Matrix<double, 16, 2>();

    W = this->state_covariance * H.transpose();

    double ic_md = innovation_covariance(0,0) * innovation_covariance(1,1);
    double ic_sd = innovation_covariance(0,1) * innovation_covariance(1,0);
    double d_diff = ic_md - ic_sd;


    /* Matrix division */
    // W = innovation_covariance.transpose().colPivHouseholderQr().solve(W.transpose()).transpose();  
    W = W*innovation_covariance.inverse();

    this->filter_state = this->filter_state + W * innovation;
    this->state_covariance = this->state_covariance - W * H * this->state_covariance;
    // InsFilterNonHolonomicTypes::Quaternion::repair({
    //     this->filter_state(0, 0),
    //     this->filter_state(1, 0),
    //     this->filter_state(2, 0),
    //     this->filter_state(3, 0)
    // });
    double n = sqrt(pow(this->filter_state(0, 0), 2) + pow(this->filter_state(1, 0), 2) + pow(this->filter_state(2, 0), 2) + pow(this->filter_state(3, 0), 2));
    this->filter_state(0, 0) /= n;
    this->filter_state(1, 0) /= n; 
    this->filter_state(2, 0) /= n;
    this->filter_state(3, 0) /= n;

    if (this->filter_state(0, 0) < 0)
    {
        this->filter_state(0, 0) *= -1;
        this->filter_state(1, 0) *= -1; 
        this->filter_state(2, 0) *= -1;
        this->filter_state(3, 0) *= -1;
    }

    MatrixXd tmp = innovation.transpose();
    innovation = tmp;
}

void InsFilterNonHolonomic::predict(Vector3d accel_data, Vector3d gyro_data, double delta_time)
{
    this->dt = delta_time;

    /* Extended Kalman filter predict algorithm */
    this->setState(accel_data, gyro_data);

    MatrixXd F = this->stateTransitionJacobianFcn(accel_data, gyro_data);
    MatrixXd G = this->processNoiseJacobianFcn();
    MatrixXd U = this->processNoiseCovariance();
    
    /* Process new state covariance */
    this->setStateCovariance(F, U, G);

    /* Apply vehicle kinematic constraints */
    this->applyConstraintCount++;
    if(this->applyConstraintCount == this->decimation_factor)
    {
        this->correctKinematics();
        this->applyConstraintCount = 0;
    }

    // this->printCurrentState();

}

/* ***** END PREDICT METHODS ***** */

/* ***** CORRECT METHODS ***** */

Vector3d lla2ecef(Vector3d lla)
{
    /* Semi-major axis (m) */
    double a = 6378137.0;
    /* Flattening */
    double f = (1.0f / 298.257223563);

    double phi = lla[0];
    double lambda = lla[1];
    double h = lla[2];

    double sinphi = sind(phi);
    double cosphi = cosd(phi);
    double e2 = f * (2 - f);
    double N  = a / sqrt(1 - e2 * pow(sinphi,2));
    double rho = (N + h) * cosphi;
    double z = (N*(1 - e2) + h) * sinphi;
    double x = rho * cosd(lambda);
    double y = rho * sind(lambda);

    Vector3d ecefPos = Vector3d(x, y, z);
    
    return ecefPos;
}

Vector3d ecef2enu(Vector3d ecefPos, Vector3d lla)
{
    double phi = lla[0];
    double lambda = lla[1];

    double cosphi = cosd(phi);
    double sinphi = sind(phi);
    double coslambda = cosd(lambda);
    double sinlambda = sind(lambda);

    Vector3d ecef0 = lla2ecef(lla);

    /* Computing the difference in the ECEF frame for now,
     * oblateSpheroid/ecefOffset has a different algorithm that minimizes
     * numerical round off.
    */ 

    Vector3d ecefPosWithENUOrigin = ecefPos - ecef0;
    double x = ecefPosWithENUOrigin[0];
    double y = ecefPosWithENUOrigin[1];
    double z = ecefPosWithENUOrigin[2];

    /* To rotate ECEF to ENU frame, use the transpose of the rotation matrix that
    * rotates the ENU to ECEF frame (origin is the reference LLA coordinates).
    * rotENU2ECEF = Rz(-(pi/2 + lambda)) * Ry(0) * Rx(-(pi/2 - phi))
    * rotENU2ECEF = [-sinlambda -coslambda.*sinphi coslambda.*cosphi
    *                 coslambda -sinlambda.*sinphi sinlambda.*cosphi
    *                     0            cosphi            sinphi     ];
    * rotECEF2ENU = rotENU2ECEF.';
    * rotECEF2ENU = [     -sinlambda          coslambda      0
    *                -coslambda.*sinphi -sinlambda.*sinphi cosphi
    *                 coslambda.*cosphi  sinlambda.*cosphi sinphi];
    */

    double uEast = -sinlambda * x + coslambda * y;
    double tmp = coslambda * x + sinlambda * y;
    double vNorth = -sinphi * tmp + cosphi * z;
    double wUp    =  cosphi * tmp + sinphi * z;

    Vector3d enuPos = Vector3d(uEast, vNorth, wUp);

    return enuPos;
}

MatrixXd InsFilterNonHolonomic::getPosInNEDFormat(Vector3d lla)
{
    /* Semi-major axis (m) */
    double a = 6378137.0;
    /* Flattening */
    double f = (1.0f / 298.257223563);
    double phi = lla[0];
    double lambda = lla[1];
    double h = lla[2];
    double sinphi = sind(phi);
    double cosphi = cosd(phi);
    double e2 = f * (2 - f);
    double N  = a / sqrt(1 - e2 * pow(sinphi,2));
    double rho = (N + h) * cosphi;
    double z = (N*(1 - e2) + h) * sinphi;
    double x = rho * cosd(lambda);
    double y = rho * sind(lambda);

    Vector3d ecefPos = Vector3d(x, y, z);
    Vector3d enuPos = ecef2enu(ecefPos, Vector3d(this->reference_location.latitude, this->reference_location.longitude, this->reference_location.altitude));

    MatrixXd nedPos = Matrix<double, 3, 1>(enuPos[1], enuPos[0], enuPos[2]);

    return nedPos;
}

void InsFilterNonHolonomic::velAndCovToCourseAndCov(
    Vector2d vel, 
    Matrix3d vel_r, 
    double &course, 
    double &courseR)
{
    double groundspeed = sqrt(pow(vel[0], 2) + pow(vel[1], 2));
    double groundspeedR = 0.0; //norm(velR(1:2,1:2), 'fro');

    courseR = groundspeedR / (pow(groundspeed,2));

    /**
     * Always use the y- and x-coordinate as inputs 1 and 2,
     * respectively, since this will be compared against the current
     * heading estimate, which is 0 whenever the body x-axis is
     * aligned with the navigation x-axis.
    */
    course = atan2(vel[1], vel[0]);
    while (course < 0)
        course = course + 2*M_PI;

}


MatrixXd InsFilterNonHolonomic::measurementFcnGPS()
{
    MatrixXd h = Matrix<double, 4, 1>();

    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    // double gbX = this->filter_state(4, 0);
    // double gbY = this->filter_state(5, 0);
    // double gbZ = this->filter_state(6, 0);
    double pn = this->filter_state(7, 0);
    double pe = this->filter_state(8, 0);
    double pd = this->filter_state(9, 0);
    // double vn = this->filter_state(10, 0);
    // double ve = this->filter_state(11, 0);
    // double vd = this->filter_state(12, 0);
    // double abX = this->filter_state(13, 0);
    // double abY = this->filter_state(14, 0);
    // double abZ = this->filter_state(15, 0);

    h(0,0) = pn;
    h(1,0) = pe;
    h(2,0) = pd;
    h(3,0) = atan2((q0*q3*2 + q1*q2*2),(pow(q0,2)*2 - 1 + pow(q1,2)*2));

    return h;
}

MatrixXd InsFilterNonHolonomic::measurementJacobianFcnGPS()
{
    MatrixXd H = Matrix<double, 4, 16>();
    /* Retrieve actual filter state variables */
    double q0 = this->filter_state(0, 0);
    double q1 = this->filter_state(1, 0);
    double q2 = this->filter_state(2, 0);
    double q3 = this->filter_state(3, 0);
    // double gbX = this->filter_state(4, 0);
    // double gbY = this->filter_state(5, 0);
    // double gbZ = this->filter_state(6, 0);
    // double pn = this->filter_state(7, 0);
    // double pe = this->filter_state(8, 0);
    // double pd = this->filter_state(9, 0);
    // double vn = this->filter_state(10, 0);
    // double ve = this->filter_state(11, 0);
    // double vd = this->filter_state(12, 0);
    // double abX = this->filter_state(13, 0);
    // double abY = this->filter_state(14, 0);
    // double abZ = this->filter_state(15, 0);

    H(0, 0) = 0;
    H(0, 1) = 0;
    H(0, 2) = 0;
    H(0, 3) = 0;
    H(0, 4) = 0;
    H(0, 5) = 0;
    H(0, 6) = 0;
    H(0, 7) = 1;
    H(0, 8) = 0;
    H(0, 9) = 0;
    H(0, 10) = 0;
    H(0, 11) = 0;
    H(0, 12) = 0;
    H(0, 13) = 0;
    H(0, 14) = 0;
    H(0, 15) = 0;

    H(1, 0) = 0;
    H(1, 1) = 0;
    H(1, 2) = 0;
    H(1, 3) = 0;
    H(1, 4) = 0;
    H(1, 5) = 0;
    H(1, 6) = 0;
    H(1, 7) = 0;
    H(1, 8) = 1;
    H(1, 9) = 0;
    H(1, 10) = 0; 
    H(1, 11) = 0; 
    H(1, 12) = 0; 
    H(1, 13) = 0; 
    H(1, 14) = 0; 
    H(1, 15) = 0;

    H(2, 0) = 0;
    H(2, 1) = 0;
    H(2, 2) = 0;
    H(2, 3) = 0;
    H(2, 4) = 0;
    H(2, 5) = 0;
    H(2, 6) = 0;
    H(2, 7) = 0;
    H(2, 8) = 0;
    H(2, 9) = 1;
    H(2, 10) = 0; 
    H(2, 11) = 0; 
    H(2, 12) = 0; 
    H(2, 13) = 0; 
    H(2, 14) = 0; 
    H(2, 15) = 0; 

    H(3, 0) = (((2*q3)/(2*pow(q0,2) + 2*pow(q1,2) - 1) - (4*q0*(2*q0*q3 + 2*q1*q2))/pow((2*pow(q0,2) + 2*pow(q1,2) - 1),2))*pow((2*pow(q0,2) + 2*pow(q1,2) - 1),2))/(pow((2*pow(q0,2) + 2*pow(q1,2) - 1),2) + pow((2*q0*q3 + 2*q1*q2),2));
    H(3, 1) = (((2*q2)/(2*pow(q0,2) + 2*pow(q1,2) - 1) - (4*q1*(2*q0*q3 + 2*q1*q2))/pow((2*pow(q0,2) + 2*pow(q1,2) - 1),2))*pow((2*pow(q0,2) + 2*pow(q1,2) - 1),2))/(pow((2*pow(q0,2) + 2*pow(q1,2) - 1),2) + pow((2*q0*q3 + 2*q1*q2),2));
    H(3, 2) = (2*q1*(2*pow(q0,2) + 2*pow(q1,2) - 1))/(pow((2*pow(q0,2) + 2*pow(q1,2) - 1),2) + pow((2*q0*q3 + 2*q1*q2),2));
    H(3, 3) = (2*q0*(2*pow(q0,2) + 2*pow(q1,2) - 1))/(pow((2*pow(q0,2) + 2*pow(q1,2) - 1),2) + pow((2*q0*q3 + 2*q1*q2),2));
    H(3, 4) = 0;
    H(3, 5) = 0;
    H(3, 6) = 0;
    H(3, 7) = 0;
    H(3, 8) = 0;
    H(3, 9) = 0;
    H(3, 10) = 0;
    H(3, 11) = 0;
    H(3, 12) = 0;
    H(3, 13) = 0;
    H(3, 14) = 0;
    H(3, 15) = 0;

    return H;

}

void InsFilterNonHolonomic::correctEqn(MatrixXd h,
    MatrixXd H,
    MatrixXd z,
    Matrix4d R
)
{
    MatrixXd innovation_covariance = H * this->state_covariance * H.transpose() + R;
    MatrixXd innovation = z - h;
    MatrixXd W = Matrix<double, 16, 2>();

    W = this->state_covariance * H.transpose();

    /* Matrix division */
    // W = innovation_covariance.transpose().colPivHouseholderQr().solve(W.transpose()).transpose();  
    W = W*innovation_covariance.inverse();

    this->filter_state = this->filter_state + W * innovation;
    this->state_covariance = this->state_covariance - W * H * this->state_covariance;

    double n = sqrt(pow(this->filter_state(0, 0), 2) + pow(this->filter_state(1, 0), 2) + pow(this->filter_state(2, 0), 2) + pow(this->filter_state(3, 0), 2));
    this->filter_state(0, 0) /= n;
    this->filter_state(1, 0) /= n; 
    this->filter_state(2, 0) /= n;
    this->filter_state(3, 0) /= n;

    if (this->filter_state(0, 0) < 0)
    {
        this->filter_state(0, 0) *= -1;
        this->filter_state(1, 0) *= -1; 
        this->filter_state(2, 0) *= -1;
        this->filter_state(3, 0) *= -1;
    }
    MatrixXd tmp = innovation.transpose();
    innovation = tmp;

}

void InsFilterNonHolonomic::fusegps(Vector3d lla_position, Vector3d velocity)
{
    Matrix3d pos_covariance = Matrix<double, 3,3>::Identity() * this->r_pos;
    Matrix3d vel_covariance = Matrix<double, 3,3>::Identity() * this->r_vel;

    MatrixXd h = this->measurementFcnGPS();
    MatrixXd H = this->measurementJacobianFcnGPS();

    double course, course_r;

    this->velAndCovToCourseAndCov(Vector2d(velocity[0], velocity[1]), vel_covariance, course, course_r);

    /**
     * Adjust measured course so that the magnitude of the 
     * difference between it and the estimated course is less than
     * or equal to 180 degrees
    */
    double courseDiff = course - h(3,0);
    if (courseDiff > M_PI)
        course = course - (2*M_PI);

    MatrixXd pos = this->getPosInNEDFormat(lla_position);
    
    MatrixXd z = Matrix<double, 4, 1>(pos(0,0), pos(1,0), pos(2,0), course);

    Matrix4d R = Matrix4d::Identity() * this->r_pos;
    R(3,3) = course_r;

    /* Updated filter state and state covariance */
    this->correctEqn(h, H, z, R);
}

/* ***** END CORRECT METHODS ***** */

void InsFilterNonHolonomic::pose(Vector3d &curr_position, Vector4d &curr_orientation, 
    Vector3d &curr_velocity)
{
    /* Set actual orientation */
    curr_orientation = Vector4d(this->filter_state(0,0), this->filter_state(1,0),this->filter_state(2,0), this->filter_state(3,0));

    /* Set actual position */
    curr_position = Vector3d(this->filter_state(7,0),this->filter_state(8,0), this->filter_state(9,0));

    /* Set actual velocities */
    curr_velocity = Vector3d(this->filter_state(10,0),this->filter_state(11,0), this->filter_state(12,0));
}

void InsFilterNonHolonomic::printFilterConstraints()
{
    std::cout << "gyroscope_noise: " << this->gyroscope_noise << "\n";
    std::cout << "gyroscope_bias_noise: " << this->gyroscope_bias_noise << "\n";
    std::cout << "gyroscope_bias_decay_factor: " << this->gyroscope_bias_decay_factor << "\n";
    std::cout << "accelerometer_noise: " << this->accelerometer_noise << "\n";
    std::cout << "accelerometer_bias_noise: " << this->accelerometer_bias_noise << "\n";
    std::cout << "accel_bias_decay_factor: " << this->accel_bias_decay_factor << "\n";
    std::cout << "r_vel: " << this->r_vel << "\n";
    std::cout << "r_pos: " << this->r_pos << "\n";
    std::cout << "zero_velocity_constraint_noise: " << this->zero_velocity_constraint_noise << "\n";
}

void InsFilterNonHolonomic::printCurrentState()
{
    std::cerr << "Quaternion q0: " << this->filter_state(0,0) << "\n";
    std::cerr << "Quaternion q1: " << this->filter_state(1,0) << "\n";
    std::cerr << "Quaternion q2: " << this->filter_state(2,0) << "\n";
    std::cerr << "Quaternion q3: " << this->filter_state(3,0) << "\n";

    std::cerr << "Gyscope Bias x: " << this->filter_state(4,0) << "\n";
    std::cerr << "Gyscope Bias y: " << this->filter_state(5,0) << "\n";
    std::cerr << "Gyscope Bias z: " << this->filter_state(6,0) << "\n";

    std::cerr << "Position N: " << this->filter_state(7,0) << "\n";
    std::cerr << "Position E: " << this->filter_state(8,0) << "\n";
    std::cerr << "Position D: " << this->filter_state(9,0) << "\n";

    std::cerr << "Velocity N: " << this->filter_state(10,0) << "\n";
    std::cerr << "Velocity E: " << this->filter_state(11,0) << "\n";
    std::cerr << "Velocity D: " << this->filter_state(12,0) << "\n";

    std::cerr << "Accelerometer Bias x: " << this->filter_state(13,0) << "\n";
    std::cerr << "Accelerometer Bias y: " << this->filter_state(14,0) << "\n";
    std::cerr << "Accelerometer Bias z: " << this->filter_state(15,0) << "\n";
}

void InsFilterNonHolonomic::printCurrentStateCovariance()
{
    std::cerr << this->state_covariance << "\n\n";
}

void InsFilterNonHolonomic::printFilterNoises()
{
    std::cerr << "Gyscope Bias x: " << this->filter_state(4,0) << "\n";
    std::cerr << "Gyscope Bias y: " << this->filter_state(5,0) << "\n";
    std::cerr << "Gyscope Bias z: " << this->filter_state(6,0) << "\n";

    std::cerr << "Accelerometer Bias x: " << this->filter_state(13,0) << "\n";
    std::cerr << "Accelerometer Bias y: " << this->filter_state(14,0) << "\n";
    std::cerr << "Accelerometer Bias z: " << this->filter_state(15,0) << "\n";
}

InsFilterNonHolonomic::~InsFilterNonHolonomic()
{
    ;
}