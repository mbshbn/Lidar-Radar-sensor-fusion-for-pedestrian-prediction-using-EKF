# EKF-for-pedestrian-prediction

The goal is here to locate the pedestrian in front of a self driving car using the Radar and Lidar measurements.

In the following you see
* Kalman filter for 1D motion
* Kalman filter for 2D Lidar (Laser) measurements
* Extended Kalman filter for 2D Radar measurements
* Sensor fusion (EKF) for 2D Laser and 2D Radar measurements
* Validation of estimation using Root Mean Square Error (RSME)

### Eigen library
For this project the [Eigen library](http://eigen.tuxfamily.org/index.php?title=Main_Page), which is linear algebra library, is used. To be able to use the Eigen library you need to
* Add the path for the header files
* Add the path for the actual code (i.e. the library)

You can find instructions online, for example for [Code::Blocks IDE](https://www.learncpp.com/cpp-tutorial/a3-using-libraries-with-codeblocks/) or [CodeLite IDE](https://automaticaddison.com/how-to-add-an-external-c-library-to-your-project/).

## Kalman filter for 1D motion
The code is inside the folder called `pedestrian-prediction`.

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
Usually the parameters of `R` are provided by the sensor manufactures.
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
P = F * P * F.transpose() + Q;
```
## Kalman filter for 2D motion using Lidar (laser) measurements
The code is inside the folder called `pedestrain-prediction-2D-Lidar`.

The state vector is 4 by 1, consist of x = [pos_x, pos_y, vel_x, vel_y]^T. The following assumption is made:
* The time interval is not constant.
* A linear motion model is used. The velocity vector is not constant, i.e. acceleration is not zero. But, we only track position and velocity, so acceleration is modeled as a random noise with 0 mean and a new covariance `Qv`.
* The process noise is considered as `a dt^2/2`, where `a` is acceleration, and `dt` is the time interval. Thus the process covariance matrix consists of `dt` and `acceleration` as a random noise. It can be decomposed of two matrices, one consists of `dt`, and the other consists of random acceleration.
* Lidar data is a point cloud, and for simplicity here it is assumed that the data is analyzed and the 2D location of the pedestrian is computed.

The code consists of three classes:
* Kalman filter class (predicts and updates)
* Measurement class
* Tracking classes (call other two classes, first process the measurements, then predicts and updates)

The matrices are defined as
```
// state covariance matrix P
kf_.P_ =Eigen::MatrixXd(4, 4);
kf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;


// measurement covariance
kf_.R_ = Eigen::MatrixXd(2, 2);
kf_.R_ << 0.0225, 0,
          0, 0.0225;

// measurement matrix
kf_.H_ = Eigen::MatrixXd(2, 4);
kf_.H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

// the initial transition matrix F_
kf_.F_ = Eigen::MatrixXd(4, 4);
kf_.F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;
kf_.F_(0, 2) = dt;
kf_.F_(1, 3) = dt;
// the acceleration noise components
noise_ax = 5;
noise_ay = 5;
//  the process covariance matrix Q
kf_.Q_ = Eigen::MatrixXd(4, 4);
kf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
```
The predict function includes
```
x_ = F_ * x_;
Eigen::MatrixXd Ft = F_.transpose();
P_ = F_ * P_ * Ft + Q_;
```
The update function includes
```
Eigen::VectorXd y = z - H_ * x_;
Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

//new estimate
x_ = x_ + (K * y);
P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
```


## Extended Kalman filter for 2D motion using Radar measurements
The Radar data includes the range (the distance of the object form the origin), the radial velocity (the range rate, i.e time derivative of the range), and the bearing (the angle between x axis and the range. x axis's direction is usually in the direction of motion of the car). The angle for calculation should be between -pi and pi. So, add or subtract 2pi to have the angle between -pi and pi.

So, the data is in polar coordinate frame. Thus, the measurement function `h(x)`, the functions that maps from `x` (including position and velocity in the Cartesian frame) to the measurement data (in the polar coordinate frame), is not linear. A Gaussian distribution after applying a nonlinear function may not be Gaussian. So, we use (multi-dimensional) Tyler expansion to find the linear approximation of (multidimensional equation) `h(x)`. The second and higher order terms are negligible, and ignored. So, here only the Jacobian of `h(x)` is calculated as follows.
```
// compute the Jacobian matrix for Radar measurements
float c1 = px*px+py*py;
float c2 = sqrt(c1);
float c3 = (c1*c2);
Hj << (px/c2), (py/c2), 0, 0,
    -(py/c1), (px/c1), 0, 0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
```  
Note that `Hj` depends on `x` and have to be computed for each data points.
Generally, the EKF differs from KF as follows:
* Jacobian of f (`Fj`) is replaced with `F`
* Jacobian of f (`Hj`) is replaced with `H`
* nonlinear function of `f` is used instead of `F*x`
* nonlinear function of `h` is used instead of `H*x`

But for Radar, `f(x)=F*x` is linear, and we can use the same prediction equations as we used before.
```
VectorXd z = measurements[n];

// KF Measurement update step
VectorXd y = z - h;
MatrixXd S = Hj *P * Hj.transpose() + R;
MatrixXd K = P * Hj.transpose() * S.inverse();

// new state
x = x + (K * y);
P = (I-(K * Hj)) * P;

// KF Prediction step
x = F * x ;
P = F * P * F.transpose();
```

## Extended Kalman filter for 2D motion using Lidar & Radar measurements


### Validation using Root Mean Square Error (RSME)
In case the we have the ground truth, and we want to measure the accuracy of estimation, RMSE can be computed. Small RMSE indicated accurate estimation.
```
for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

// calculate the mean
rmse = rmse/estimations.size();

// calculate the squared root
rmse = rmse.array().sqrt();
```


This project is developed based on the [SELF-DRIVING CAR ENGINEER NANODEGREE PROGRAM](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).
