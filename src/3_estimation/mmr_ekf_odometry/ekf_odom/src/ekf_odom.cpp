#include <ekf_odom.hpp>


#include <omp.h>
#include <chrono>

/* Constructor */
EKFOdom::EKFOdom(Vector2f process_noise, Vector3f measurement_noise, const float alpha)
{
    /* Init number of threads */
    omp_set_num_threads(6);
    Eigen::setNbThreads(6);

    std::cerr << "Num threads: " << Eigen::nbThreads() << "\n";
    
    /* Initialize state vector */
    this->x_ = VectorXf::Zero(2 * N_CONES + 3);

    /* Initialize covariance matrix */
    this->P_ = MatrixXf::Zero(2 * N_CONES + 3, 2 * N_CONES + 3);
    // this->P_.setConstant(INF);
    this->P_.block(0, 0, 3, 3) = Matrix3f::Identity();
    this->P_.bottomRightCorner(2 * N_CONES, 2 * N_CONES) = INF * Eigen::MatrixXf::Identity(2 * N_CONES, 2 * N_CONES);

    /* Initialize Process noise covariance */
    this->Q_ = Matrix2f::Identity();
    for (uint8_t i=0;i < process_noise.size(); i++)
    {
        // this->Q_(i,i) = pow(process_noise(i), 2);
        this->Q_(i,i) = process_noise(i);
    }
    std::cerr << "Q_ : \n" << this->Q_ << "\n";

    /* Initialize Measurement noise covariance */
    this->R_ = Matrix3f::Identity();
    for (uint8_t i=0;i < measurement_noise.size(); i++)
    {
        // this->R_(i,i) = pow(measurement_noise(i), 2);
        this->R_(i,i) = measurement_noise(i);
    }

    std::cerr << "R_ : \n" << this->R_ << "\n";


    /* Initialize max new cone dist */
    this->max_new_cone_dist = alpha;

    /* Initialize Fx */
    this->Fx_ = MatrixXf::Zero(3, 2*N_CONES+3);
    this->Fx_.block(0,0,3,3) = Matrix3f::Identity();

    /* Initialize Fx_k*/
    this->Fx_k = MatrixXf::Zero(5, 2*N_CONES+3);
    this->Fx_k.block(0,0,3,3) = Matrix3f::Identity();
}


/* EKF Predict step function */
void EKFOdom::predict(const float dt) {
    /* If, for any reason, actual ang velocity is zero, set a very tiny quanity to avoid a division by 0 */
    this->act_ang_vel = (this->act_ang_vel == 0.0) ? __FLT_MIN__ : this->act_ang_vel;

    Matrix3f gt = Matrix3f::Zero();
    gt(0,2) = 
        (this->act_vel / this->act_ang_vel) * (sin(this->x_(2) + this->act_ang_vel * dt) - sin(this->x_(2)));
    gt(1,2) = 
        (this->act_vel / this->act_ang_vel) * (cos(this->x_(2)) - cos(this->x_(2) + this->act_ang_vel * dt));
    gt(2,2) = 1.0f;

    MatrixXf Gt = MatrixXf::Identity(2*N_CONES + 3, 2*N_CONES + 3);
    Gt = Gt + (this->Fx_.transpose() * gt * this->Fx_);
    // Gt.bottomRightCorner(2 * N, 2 * N) = Eigen::MatrixXf::Identity(2 * N, 2 * N);

    Vector3f velocity_based_motion;
    velocity_based_motion(0) = 
        (-this->act_vel / this->act_ang_vel) * sin(this->x_(2)) + 
        (this->act_vel / this->act_ang_vel) * sin(this->x_(2) + this->act_ang_vel * dt);
    velocity_based_motion(1) = 
        (this->act_vel / this->act_ang_vel) * cos(this->x_(2)) - 
        (this->act_vel / this->act_ang_vel) * cos(this->x_(2) + this->act_ang_vel * dt);
    velocity_based_motion(2) =
        this->act_ang_vel * dt;

    /* Update expected vehicle state */
    this->x_ += this->Fx_.transpose() * velocity_based_motion;
    // this->x_(2) = this->normalizeYaw(this->x_(2));

    /* Update expected covariance matrix */
    this->P_ = (Gt * this->P_ * Gt.transpose()) + (this->Fx_.transpose() * this->R_ * this->Fx_);
}


/* EKF Correct step function */
void EKFOdom::correct(const Vector3f *z, const size_t act_cones_detected) {    
    /* Vector that stores the temp parameters for a hypothetical new cone */
    Vector2f tmp_cone;
    /* Vector that stores the delta X,Y between a cone and the vehicle */
    Vector2f delta_k;

    size_t i,k, j = 0;
    float min_dist;

    // VectorXf new_state_update = VectorXf::Zero(2*N_CONES+3);
    // MatrixXf new_covariance_update = MatrixXf::Zero(2*N_CONES+3, 2*N_CONES+3);

    MatrixXf Ip = MatrixXf::Identity(2*N_CONES + 3, 2*N_CONES + 3);

    for(i = 0; i < act_cones_detected; i++)
    {
        /* Discard orange cones */
        if ((z[i][2] == 2) || (z[i][2] == 3))
        {
            // std::cerr << "Skipping orange cone.\n";
            continue;   
        }

        bool is_new_cone = false;
        /* Reset min distance and k index */
        min_dist = __FLT_MAX__;
        k = 0;

        /* Compute expected NEW cone position */
        tmp_cone << this->x_(0), this->x_(1);
        tmp_cone = tmp_cone + (z[i](0) * Vector2f(cos(z[i](1) + this->x_(2)), sin(z[i](1) + this->x_(2))));

        // std::cerr << "EKF INDEX: " << i << " TMP CONE X: " << tmp_cone(0) << " TMP CONE Y: " << tmp_cone(1) << "\n";

        /* Check if this cone has been seen before based on nearest neightbour (min distance) */

        for (j = 0; j < this->landmark_count; j++)
        {
            float tempDistance = euclideanDistance(this->x_(j * 2 + 3), tmp_cone(0), this->x_(j * 2 + 4), tmp_cone(1));
            // // std::cerr << "Distance between cone n° "<<j<<" ("<<this->x_(j * 2 + 3)<<","<<this->x_(j * 2 + 4)<<
            //     ") and new cone ("<<tmp_cone(0)<<","<<tmp_cone(1)<<") is: " << tempDistance <<"\n";
            if (tempDistance < min_dist)
            {
                min_dist = tempDistance;
                k = j;
            }
        }

        if(this->max_new_cone_dist < min_dist)
        {
            /* If this is not the first lap, skip to next observation */
            if (this->is_first_lap_completed)
            {
                continue;
            }

            /* Discard orange cones */
            if ((z[i][2] == 2) || (z[i][2] == 3))
            {
                // std::cerr << "Skipping orange cone.\n";
                continue;   
            }
            /* New cone */
            is_new_cone = true;

            // std::cerr << "EKF INDEX: " << i << " NEW CONE!!\n";
            k = this->landmark_count;

            this->landmark_count++;

            /* Set initial landmark position and signature */
            this->x_.segment(3 + (2*k), 2) = tmp_cone;
            ColorLogic c;
            c.setColor(static_cast<uint32_t>(z[i][2]));
            this->s_(k) = c;
        } else {
            /* Otherwise, update color counter */
            this->s_(k).setColor(static_cast<uint32_t>(z[i][2]));
        }

        // std::cerr << "EKF INDEX: " << i << " k is: " << k << "\n";
        if (i == (act_cones_detected-1))
        {
            delta_k = this->x_.segment((3 + (2*k)), 2) - Vector2f(this->x_(0), this->x_(1)); /* This is equivalent to ((cone_x - vehicle_x), (cone_y - vehicle_y)) */            

            float q_k = delta_k.transpose() * delta_k; /* Compute euclidean distance ^2 */

            q_k = (q_k == 0.0f) ? (__FLT_MIN__) : (q_k);

            /* Compute predicted measurement */
            Vector2f exp_z_k
            (
                sqrt(q_k),
                atan2(delta_k(1), delta_k(0)) - this->x_(2)
            );

            /* Normalize exp z angle */
            exp_z_k(1) = normalizeAngle(exp_z_k(1));

            // if (!is_new_cone)
            // {
                // std::cerr << "EKF INDEX: " << i << " MEAS z is: \n" << z[i] << "\n";
                // std::cerr << "EKF INDEX: " << i << " EXPC z is: \n" << exp_z_k << "\n";
            // }

            /* Set to 1 the elements of the matrix Fx_k corresponding to the actual cone */
            this->Fx_k.setConstant(0);
            this->Fx_k.block(0,0,3,3) = Matrix3f::Identity();

            this->Fx_k.block(3,3 + 2*(k),2,2) = Matrix2f::Identity();

            MatrixXf ht_k = MatrixXf::Zero(2,5);
            ht_k(0,0) = -sqrt(q_k) * delta_k(0);
            ht_k(0,1) = -sqrt(q_k) * delta_k(1);
            ht_k(0,2) = 0.0;
            ht_k(0,3) = sqrt(q_k) * delta_k(0);
            ht_k(0,4) = sqrt(q_k) * delta_k(1);

            ht_k(1,0) = delta_k(1);
            ht_k(1,1) = -delta_k(0);
            ht_k(1,2) = -q_k;
            ht_k(1,3) = -delta_k(1);
            ht_k(1,4) = delta_k(0);

            /* Ht_k size: (2, 2N + 3)*/
            MatrixXf Ht_k;
            Ht_k = ((1 / q_k) * ht_k) * Fx_k;

            /* Compute Kalman Gain. K size: (2N+3, 2) */
            MatrixXf K;
            K = this->P_ * Ht_k.transpose() * (((Ht_k * this->P_ * Ht_k.transpose()) + this->Q_).inverse());

            Vector2f meas_diff = Vector2f(z[i](0), z[i](1)) - exp_z_k;
            /* Update filter state */
            this->x_.noalias() += (K * meas_diff);
            this->x_(2) = this->normalizeYaw(this->x_(2));
            // new_state_update = (K * meas_diff);
            // new_covariance_update = (K * Ht_k);

            /* Update filter covariance */
            Ip.noalias() -= (K * Ht_k).eval();
            this->P_ =  Ip * this->P_;

        }
    }
    // // std::cerr << "new vehicle pose: (" << this->x_(0) << ","<< this->x_(1) << "," << this->x_(2) <<")\n";
}

/* Return current filter state */
VectorXf EKFOdom::getState() const {
    return x_;
}

/* Return current error covariance */
MatrixXf EKFOdom::getCovariance() const {
    return P_;
}

Vector3f EKFOdom::getPoseCovariance() const
{
    Vector3f pose_cov;
    for (size_t i = 0; i < 3; i++)
    {
        pose_cov(i) = this->P_(i,i);
    }

    return pose_cov;
}

/* Return current process noise covariance */
Matrix2f EKFOdom::getProcessNoiseCovariance() const {
    return Q_;
}

Matrix3f EKFOdom::getMeasurementNoiseCovariance() const {
    return R_;
}

MatrixXf EKFOdom::getFx() const {
    return Fx_;
}
void EKFOdom::setFirstLapCompleted(const bool first_lap_completed)
{
    this->is_first_lap_completed = first_lap_completed;
}


void EKFOdom::setActVel(const float vel)
{
    this->act_vel = vel;
}
void EKFOdom::setActAngVel(const float ang_vel)
{
    this->act_ang_vel = ang_vel;
}

void EKFOdom::setPose(const Vector3f pose)
{
    this->x_.segment(0,3) = pose;
    this->is_pose_initialized = true;
}

void EKFOdom::setPoseCovariance(const Vector3f pos_cov)
{
    for (size_t i=0; i < 3; i++)
    {
        this->P_(i,i) += pos_cov(i);
    }
}

size_t EKFOdom::getActMappedLandmarks() const
{
    return this->landmark_count;
}

SignatureVector EKFOdom::getSignatures() const
{
    return this->s_;
}


/* Destructor */
EKFOdom::~EKFOdom()
{
    ;
}