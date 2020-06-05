# EKF-for-pedestrian-prediction

The goal is here to locate the pedestrian in front of a self driving car using the measurements.

## Kalman filter for 1D motion
The vector state is defined as x = [pos, vel]^T which contains position and velocity of a pedestrian, and the measurement data only contains a position for any given time. The following assumptions are made:
* It is assumed that the velocity remains constant between time intervals (a model uncertainty is considered), in other words a linear motion model is considered.
* Tt is assumed that the time intervals remains constant.
Tus, the state transition matrix `F`, and the measurement matrix `H` are defined as follows:
```
  F = MatrixXd(2, 2);
  F << 1, 1, 0, 1;

  H = MatrixXd(1, 2);
  H << 1, 0;
```
The state covariance matrix `P` and the measurements covariance matrix `R` (due to sensor noise), and process covariance matrix `Q` (due to model uncertainty/motion noise/process noise, for example constant velocity, and no acceleration model) are initially set to
```     
P = MatrixXd(2, 2);
P << 1000, 0, 0, 1000;
R = MatrixXd(1, 1);
R << 1;
Q = MatrixXd(2, 2);
Q << 0, 0, 0, 0;
```
The process noise depends on both the elapsed time (larger time interval, larger uncertainty) and uncertainty of acceleration (here we use a linear model with constant velocity).

The filter function updates then predicts the state vector `x` as follows for any measurement data
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
## Kalman filter for 2D motion using Lidar (laser) measurements
The state vector is 4 by 1, consist of x = [pos_x, pos_y, vel_x, vel_y]^T. The following assumption is made:
* The time interval is not constant.
* the velocity vector is not constant, i.e. acceleration is not zero. But, we only track position and velocity, so acceleration is modeled as a random noise with 0 mean and a new covariance `Qv`.
* The process noise is considered as `a dt^2/2`, where `a` is acceleration, and `dt` is the time interval. Thus the process covariance matrix consists of `dt` and `acceleration` as a random noise. It can be decomposed of two matrices, one consists of `dt`, and the other consists of random acceleration.
* Lidar data is a point cloud, and for simplicity here it is assumed that the dats is analyzed and the 2D location of the pedestrian is computed.

The code consists of three classes:
* Kalman filter class
* Tracking classes
* Measurement class
