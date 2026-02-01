#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class KalmanFilter {
public: 
    KalmanFilter(); 
    KalmanFilter(MatrixXd A,MatrixXd B,MatrixXd C,
                    MatrixXd x0,MatrixXd P0,unsigned int max_steps = 100);


private:
    int time_step; // time step
    unsigned int input_dim, state_dim, output_dim; // vector/matrix dimensions

    MatrixXd A,B,C; // system matrices (A: state transition, B: control input, C: measurement)
    MatrixXd Q,R;   // covariance matrices (process noise, measurement noise)
    MatrixXd P0;    // initial estimation error covariance
    MatrixXd x0;    // initial state 
    // keep track of states, covariances, gains, errors over time
    // A-priori: before measurement update
    // A-posteriori: after measurement update
    MatrixXd stateEstimatesApost;   // a posteriori state estimates
    MatrixXd stateEstimatesApri;    // a priori state estimates
    MatrixXd errorCovarianceApost;  // a posteriori error covariance
    MatrixXd errorCovarianceApri;   // a priori error covariance
    MatrixXd kalmanGains;           // Kalman gain matrices
    MatrixXd estimationErrors;      // estimation errors
};

#endif // KALMANFILTER_H