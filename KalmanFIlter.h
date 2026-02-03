#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <string>

using namespace Eigen;
using namespace std;

class KalmanFilter {
public: 

    KalmanFilter(MatrixXd Ainput, MatrixXd Binput, MatrixXd Hinput,
                    MatrixXd x0input, MatrixXd P0input, MatrixXd Qinput,
                    MatrixXd Rinput, unsigned int max_steps = 100);

    // predict the next state estimate and error covariance
    // from the given controls and system model
    void predictEstimate(MatrixXd controlInput);
    // update the state estimate and error covariance
    // by correcting it with the given measurement
    void updateEstimate(MatrixXd measurement);

    // load measurement data from a CSV file
    static MatrixXd readData(string readFile);
    // save the stored data to CSV files
    void saveData(string stateEstimatesApostFile, string stateEstimatesApriFile,
                  string errorCovarianceApostFile, string errorCovarianceApriFile,
                  string kalmanGainsFile, string estimationErrorsFile) const;
private:
    int time_step; // time step
    unsigned int input_dim, state_dim, output_dim; // vector/matrix dimensions

    MatrixXd A,B,H; // system matrices (A: state transition, B: control input, H: measurement)
    MatrixXd Q,R;   // covariance matrices (process noise, measurement noise)
    MatrixXd P0;    // initial estimation error covariance
    MatrixXd x0;    // initial state 
    // keep track of states, covariances, gains, errors over time
    // A-priori: before measurement update
    // A-posteriori: after measurement update
    MatrixXd stateEstimatesApost;   // a posteriori state estimates
    MatrixXd stateEstimatesApri;    // a priori state estimates
    MatrixXd stateCovarianceApost;  // a posteriori error covariance
    MatrixXd stateCovarianceApri;   // a priori error covariance
    MatrixXd kalmanGains;           // Kalman gain matrices
    MatrixXd estimationErrors;      // estimation errors
};

#endif // KALMANFILTER_H