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
    C = Cinput; 
    // initial state and covariance
    x0 = x0input;
    P0 = P0input;
    // dimensions
    input_dim = B.cols(); // control input
    state_dim = A.rows();
    output_dim = C.rows();

    // initialization of storage matrices
    stateEstimatesApost.resize(state_dim, max_steps);
    stateEstimatesApost.setZero();
    stateEstimatesApost.col(time_step) = x0;

    stateEstimatesApri.resize(state_dim, max_steps);
    stateEstimatesApri.setZero();
    stateEstimatesApri.col(time_step) = x0;

    errorCovarianceApost.resize(state_dim, state_dim * max_steps);
    errorCovarianceApost.setZero();
    errorCovarianceApost(all, seq(0, state_dim - 1)) = P0;

    errorCovarianceApri.resize(state_dim, state_dim * max_steps);
    errorCovarianceApri.setZero();
    errorCovarianceApri(all, seq(0, state_dim - 1)) = P0;

    kalmanGains.resize(state_dim, output_dim * max_steps);
    kalmanGains.setZero();

    estimationErrors.resize(output_dim, max_steps);
    estimationErrors.setZero();
}

void predictEstimate(MatrixXd controlInput){
    
}