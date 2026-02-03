#include "KalmanFilter.h"

using namespace Eigen;
using namespace Eigen::indexing;
using namespace std;

KalmanFilter::KalmanFilter(MatrixXd Ainput, MatrixXd Binput, MatrixXd Cinput,
                           MatrixXd Qinput, MatrixXd Rinput, MatrixXd P0input, 
                           MatrixXd x0input, unsigned int max_steps)
{
    time_step = 0;
    // system matrices
    A = Ainput;
    B = Binput;
    H = Cinput; 
    // noise
    Q = Qinput;
    R = Rinput;
    // initial state and covariance
    x0 = x0input;
    P0 = P0input;
    // dimensions
    input_dim = B.cols(); // control input
    state_dim = A.rows();
    output_dim = H.rows();

    cout << "Initialized system matrices and dimensions!" << endl;
    // initialization of storage matrices
    stateEstimatesApost.resize(state_dim, max_steps);
    stateEstimatesApri.setZero();
    stateEstimatesApost.col(time_step) = x0;

    cout << "Initialized stateEstimatesApost!" << endl;

    stateEstimatesApri.resize(state_dim, max_steps);
    stateEstimatesApri.setZero();
    stateEstimatesApri.col(time_step) = x0;

    cout << "Initialized stateEstimatesApri!" << endl;

    stateCovarianceApost.resize(state_dim, state_dim * max_steps);
    stateCovarianceApost.setZero();
    stateCovarianceApost(all, seq(0, state_dim - 1)) = P0;

    cout << "Initialized stateCovarianceApost!" << endl;

    stateCovarianceApri.resize(state_dim, state_dim * max_steps);
    stateCovarianceApri.setZero();
    stateCovarianceApri(all, seq(0, state_dim - 1)) = P0;

    cout << "Initialized stateCovarianceApri!" << endl;

    kalmanGains.resize(state_dim, output_dim * max_steps);
    kalmanGains.setZero();

    cout << "Initialized kalmanGains!" << endl;

    estimationErrors.resize(output_dim, max_steps);
    estimationErrors.setZero();

    cout << "Initialized estimationErrors!" << endl;
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

MatrixXd KalmanFilter::readData(string fileToOpen)
{
	// https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
	vector<double> matrixEntries;

	// in this object we store the data from the matrix
	ifstream matrixDataFile(fileToOpen);

	// this variable is used to store the row of the matrix that contains commas 
	string matrixRowString;

	// this variable is used to store the matrix entry;
	string matrixEntry;

	// this variable is used to track the number of rows
	int matrixRowNumber = 0;


	while (getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
	{
		stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.

		while (getline(matrixRowStringStream, matrixEntry,',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
		{
			matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
			}
		matrixRowNumber++; //update the column numbers
	}

	// here we convert the vector variable into the matrix and return the resulting object, 
	// note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
	return Map<Matrix<double, Dynamic, Dynamic, RowMajor>> (matrixEntries.data(), 
															matrixRowNumber, matrixEntries.size() / matrixRowNumber);

}



void KalmanFilter::saveData(string estimatesAposterioriFile, string estimatesAprioriFile, 
							string covarianceAposterioriFile, string covarianceAprioriFile, 
							string gainMatricesFile, string errorsFile) const
{
	const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
	
	ofstream file1(estimatesAposterioriFile);
	if (file1.is_open())
	{
		file1 << stateEstimatesApost.format(CSVFormat);
		
		file1.close();
	}

	ofstream file2(estimatesAprioriFile);
	if (file2.is_open())
	{
		file2 << stateEstimatesApri.format(CSVFormat);
		file2.close();
	}
	
	ofstream file3(covarianceAposterioriFile);
	if (file3.is_open())
	{
		file3 << stateCovarianceApost.format(CSVFormat);
		file3.close();
	}

	ofstream file4(covarianceAprioriFile);
	if (file4.is_open())
	{
		file4 << stateCovarianceApri.format(CSVFormat);
		file4.close();
	}

	ofstream file5(gainMatricesFile);
	if (file5.is_open())
	{
		file5 << kalmanGains.format(CSVFormat);
		file5.close();
	}

	ofstream file6(errorsFile);
	if (file6.is_open())
	{
		file6 << estimationErrors.format(CSVFormat);
		file6.close();
	}

	
}
