//
// Created by cobot on 19-7-27.
//

#ifndef ROBOTBASE_PLAYER_EKF_FUSION_H
#define ROBOTBASE_PLAYER_EKF_FUSION_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <libplayercore/playercore.h>

double pfmod(double x, double y)
{
    double r;
    r = fmod(x, y);
    return r < 0.0 ? r + y : r;
}


double fixAngle(double angle){
    return pfmod(angle,2*M_PI);
}
class Fusion {
private:
    /*!
     * \brief:state is state vector,represent x,y,roll,linear velocity of x-aixs,angular velocity of z-axis
     */
    Eigen::Matrix<double, 5, 1> X_k_;
    /*!
     * \brief:Pk_ is the  noise in estimate mu, (the prediction error)
     * Q_ is noise in noise in update step noise
     */
    Eigen::Matrix<double, 5, 5> P_k_, Q_;
    /*!
     * \brief:Obs_ is the observation vector ,represent linear velocity of odometry in x-axis,angular velocity of odometry in z-axis,
     * angular velocity of imu in z-axis,
     */
    Eigen::Matrix<double, 3, 1> Z_k_;
    /*!
     * \brief:H_ is observation matrix
     */
    Eigen::Matrix<double, 3, 5> C_;
    /*!
     * \brief:R_ is sensor noise in odometry and imu
     */
    Eigen::Matrix<double, 3, 3> R_;

    /*!
     * :\brief:A_ is ???
     */
    Eigen::Matrix<double, 5, 5> A_;

    /*!
     * \brief:measurement_residual is the distance between measurement and real value
     */
    Eigen::Vector3d measurement_residual;

    /*!
     * \brief:residual_covariance is residual covariance matrix
     */
    Eigen::Matrix<double, 3, 3> residual_covariance;

    /*!
     * \brief:K_ is Kalman Filter Gains
     */

    Eigen::Matrix<double, 5, 3> K_;

    /*!
     * \brief:I_ is unit Matrix
     */
    Eigen::Matrix<double, 5, 5> I_ = Eigen::Matrix<double, 5, 5>::Identity();


public:
    Fusion() {
        X_k_ << 0, 0, 0, 0, 0;

        P_k_ << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;

        Z_k_ << 0, 0, 0;

        Q_ = P_k_;

        R_ << 0.1, 0, 0,
                0, 2, 0,
                0, 0, 0.02;

        C_ << 0, 0, 0, 1, 0,
                0, 0, 0, 0, 1,
                0, 0, 0, 0, 1;
    }

    void update(player_position2d_data_t &pos_data, double imu_vth, double dt) {
    //std::cout<<X_k_<<std::endl;
        pos_data.pos.px = X_k_(0, 0);
        pos_data.pos.py = X_k_(1, 0);
        pos_data.pos.pa = fixAngle(X_k_(2, 0));

        //pos_data.vel.px = X_k_(3, 0);
        pos_data.vel.pa = X_k_(4, 0);

        Z_k_ << pos_data.vel.px, pos_data.vel.pa, imu_vth;

        X_k_ << pos_data.pos.px + pos_data.vel.px * cos(pos_data.pos.pa) * dt,
                pos_data.pos.py + pos_data.vel.px * sin(pos_data.pos.pa) * dt,
                pos_data.pos.pa + pos_data.vel.pa * dt,
                pos_data.vel.px,
                pos_data.vel.pa;
       // std::cout<<X_k_<<std::endl;

        A_ << 1, 0, -pos_data.vel.px * dt * sin(pos_data.pos.pa), dt * cos(pos_data.pos.pa), 0,
                0, 1, -pos_data.vel.px * dt * cos(pos_data.pos.pa), dt * sin(pos_data.pos.pa), 0,
                0, 0, 1, 0, dt,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;

        P_k_ = A_ * P_k_ * A_.transpose() + Q_;

        measurement_residual = Z_k_ - C_ * X_k_; //残差

        residual_covariance = C_ * P_k_ * C_.transpose() + R_;

        K_ = P_k_ * C_.transpose() * residual_covariance.inverse(); //增益

        X_k_ = X_k_ + K_ * measurement_residual; // 更新状态

        P_k_ = (I_ - K_ * C_) * P_k_; //更新误差协方差矩阵



        pos_data.pos.px = X_k_(0, 0);
        pos_data.pos.py = X_k_(1, 0);
        pos_data.pos.pa = fixAngle(X_k_(2, 0));

        pos_data.vel.px = X_k_(3, 0);
        pos_data.vel.pa = X_k_(4, 0);
    }
};

#endif //ROBOTBASE_PLAYER_EKF_FUSION_H
