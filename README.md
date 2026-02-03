# Kalman Filter C++ Implementation

A high-performance C++ implementation of the Kalman Filter algorithm for state estimation and filtering linear dynamical systems. This implementation uses the **Eigen C++ library** for efficient matrix operations.

## Overview

The Kalman Filter is an optimal recursive algorithm that estimates the state of a linear dynamic system from a series of noisy measurements.

## Class Structure

### `KalmanFilter` Class

#### Constructor
```cpp
KalmanFilter(MatrixXd Ainput, MatrixXd Binput, MatrixXd Hinput,
             MatrixXd x0input, MatrixXd P0input, MatrixXd Qinput,
             MatrixXd Rinput, unsigned int max_steps = 100);
```

**Parameters:**
- `Ainput`: State transition matrix (n × n)
- `Binput`: Control input matrix (n × m)
- `Hinput`: Measurement matrix (p × n)
- `x0input`: Initial state estimate (n × 1)
- `P0input`: Initial estimation error covariance (n × n)
- `Qinput`: Process noise covariance (n × n)
- `Rinput`: Measurement noise covariance (p × p)
- `max_steps`: Maximum number of time steps (default: 100)

Where:
- `n` = state dimension
- `m` = control input dimension
- `p` = measurement/output dimension

#### Public Methods

**`void predictEstimate(MatrixXd controlInput)`**
- Performs the prediction step of the Kalman Filter
- Must be called before each measurement update

**`void updateEstimate(MatrixXd measurement)`**
- Performs the measurement update step
- Uses current measurement to refine state estimate

**`static MatrixXd readData(string readFile)`**
- Loads measurement data from a CSV file
- Parses comma-separated values and returns as Eigen matrix
- Useful for batch processing of recorded measurements

**`void saveData(string estimatesApostFile, string estimatesApriFile, ...)`**
- Exports filter results to CSV files
- Saves:
  - A posteriori state estimates
  - A priori state estimates
  - A posteriori error covariance
  - A priori error covariance
  - Kalman gain matrices
  - Estimation errors (measurement residuals)

## System Model

The Kalman Filter assumes a linear state-space model:

**State equation (discrete):**
$$x_k = A \cdot x_{k-1} + B \cdot u_k + w_k$$

**Measurement equation:**
$$z_k = H \cdot x_k + v_k$$

Where:
- `x_k`: State vector at time k
- `u_k`: Control input
- `z_k`: Measurement/observation
- `w_k`: Process noise ~ N(0, Q)
- `v_k`: Measurement noise ~ N(0, R)
- `A`: State transition matrix
- `B`: Control input matrix
- `H`: Measurement/output matrix
- `Q`: Process noise covariance
- `R`: Measurement noise covariance

## Usage Example

```cpp
#include "KalmanFilter.h"

// Define system matrices (3-state system example)
MatrixXd A(3, 3);  // State transition
MatrixXd B(3, 1);  // Control input (zero in this case)
MatrixXd H(1, 3);  // Measurement matrix
MatrixXd Q(3, 3);  // Process noise covariance
MatrixXd R(1, 1);  // Measurement noise covariance
MatrixXd P0 = MatrixXd::Identity(3, 3);  // Initial covariance
MatrixXd x0(3, 1); x0.setZero();  // Initial state

// Create Kalman filter instance
KalmanFilter kf(A, B, H, x0, P0, Q, R, 100);

// Prediction step
MatrixXd control(1, 1);
control << 0;
kf.predictEstimate(control);

// Update step with measurement
MatrixXd measurement(1, 1);
measurement << 5.2;
kf.updateEstimate(measurement);

// Save results
kf.saveData("estimates_post.csv", "estimates_prior.csv",
            "cov_post.csv", "cov_prior.csv",
            "gains.csv", "errors.csv");
```

## Output Data

All outputs are saved as CSV files with comma-separated values:

- **estimatesAposteriori.csv**: State estimates after measurement updates (state_dim × time_steps)
- **estimatesApriori.csv**: State estimates before measurement updates (state_dim × time_steps)
- **covarianceAposteriori.csv**: Error covariance after updates (state_dim × state_dim·time_steps)
- **covarianceApriori.csv**: Error covariance before updates (state_dim × state_dim·time_steps)
- **gainMatrices.csv**: Kalman gain matrices over time (state_dim × output_dim·time_steps)
- **estimationErrors.csv**: Measurement residuals (output_dim × time_steps)

## Dependencies

- **Eigen 5.0.0** - Header-only C++ linear algebra library

## Building

Using g++:
```bash
g++ -fdiagnostics-color=always -g <your_file.cpp> KalmanFilter.cpp KalmanFilter.h \
    -o <output> -I /path/to/eigen
```
