#include "KalmanFIlter.h"

using namespace Eigen;
using namespace Eigen::indexing;
using namespace std;

KalmanFilter::KalmanFilter(MatrixXd Ainput, MatrixXd Binput, MatrixXd Cinput,
                           MatrixXd x0input, MatrixXd P0input, MatrixXd Qinput,
                           MatrixXd Rinput, unsigned int max_steps)
{
    time_step = 0;
    // system matrices
    A = Ainput;
    B = Binput;
    H = Cinput; 
    // initial state and covariance
    x0 = x0input;
    P0 = P0input;
    // dimensions
    input_dim = B.cols(); // control input
    state_dim = A.rows();
    output_dim = H.rows();

    // initialization of storage matrices
    stateEstimatesApost.resize(state_dim, max_steps);
    stateEstimatesApost.setZero();
    stateEstimatesApost.col(time_step) = x0;

    stateEstimatesApri.resize(state_dim, max_steps);
    stateEstimatesApri.setZero();
    stateEstimatesApri.col(time_step) = x0;

    stateCovarianceApost.resize(state_dim, state_dim * max_steps);
    stateCovarianceApost.setZero();
    stateCovarianceApost(all, seq(0, state_dim - 1)) = P0;

    stateCovarianceApri.resize(state_dim, state_dim * max_steps);
    stateCovarianceApri.setZero();
    stateCovarianceApri(all, seq(0, state_dim - 1)) = P0;

    kalmanGains.resize(state_dim, output_dim * max_steps);
    kalmanGains.setZero();

    estimationErrors.resize(output_dim, max_steps);
    estimationErrors.setZero();
}

void KalmanFilter::predictEstimate(MatrixXd controlInput){
    
    stateEstimatesApri.col(time_step) = A * stateEstimatesApost.col(time_step) + B * controlInput;
    MatrixXd prevApostCov = stateCovarianceApost(all, seq(time_step*state_dim, (time_step+1)*state_dim -1));
    stateCovarianceApri(all, seq(time_step*state_dim, (time_step+1)*state_dim -1)) = A * prevApostCov * A.transpose() + Q;
    time_step++;
}

void KalmanFilter::updateEstimate(MatrixXd measurement){

    MatrixXd covApri = stateCovarianceApri(all, seq((time_step-1)*state_dim, time_step*state_dim -1));
    // calculate Kalman gain
    MatrixXd kalman_gain = H * covApri * H.transpose() + R;
    kalman_gain = kalman_gain.inverse();
    kalman_gain = covApri * H.transpose() * kalman_gain;
    // save Kalman gain
    kalmanGains(all, seq((time_step-1)*output_dim, time_step*output_dim -1)) = kalman_gain;
    // update state estimate and error covariance
    estimationErrors.col(time_step-1) = measurement - H * stateEstimatesApri.col(time_step-1);
    // state
    stateEstimatesApost.col(time_step) = stateEstimatesApri.col(time_step-1) + kalman_gain * estimationErrors.col(time_step-1);
    // covariance
    MatrixXd Istate = MatrixXd::Identity(state_dim, state_dim);
    MatrixXd covApost = (Istate - kalman_gain * H) * covApri;
    stateCovarianceApost(all, seq(time_step*state_dim, (time_step+1)*state_dim -1)) = covApost;
}