# EKF-for-pedestrian-prediction

The goal is here to locate the pedestrian using the measurements.

## 1D
The vector state is defined as x = [pos, vel]^T, and the measurement data only contains position. So, matrix `F`, and `H` are defined as follows:
```
  F = MatrixXd(2, 2);
  F << 1, 1, 0, 1;

  H = MatrixXd(1, 2);
  H << 1, 0;
```
The state covariance matrix `P` and the measurements covariance `R`, and `Q` are initially set to
```     
P = MatrixXd(2, 2);
P << 1000, 0, 0, 1000;
R = MatrixXd(1, 1);
R << 1;
Q = MatrixXd(2, 2);
Q << 0, 0, 0, 0;
```
The filter function, first update then predicts as follows
```
VectorXd z = measurements[n];

// KF Measurement update step
VectorXd y = z - (H*x);
MatrixXd S = H *P * H.transpose() + R;
MatrixXd K = P * H.transpose() * S.inverse();

// new state
x = x + (K * y);
P = (I-(K * H)) * P;

// KF Prediction step
x = (F * x) + u;
P = F * P * F.transpose();
```
