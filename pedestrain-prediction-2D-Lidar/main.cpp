#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "tracking.h"

//using Eigen::MatrixXd;
//using Eigen::VectorXd;
//using std::ifstream;
//using std::istringstream;
//using std::string;
//using std::vector;


int main() {

  /**
   * Set Measurements
   */
  std::vector<MeasurementPackage> measurement_pack_list;

  // hardcoded input file with laser and radar measurements
  std::string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
  std::ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

  if (!in_file.is_open()) {
    std::cout << "Cannot open input file: " << in_file_name_ << std::endl;
  }

  std::string line;
  // set i to get only first 3 measurments
  int i = 0;
  while (getline(in_file, line) && (i<=3)) {

    MeasurementPackage meas_package;

    std::istringstream iss(line);
    std::string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    int64_t timestamp;
    if (sensor_type.compare("L") == 0) {  // laser measurement
      // read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = Eigen::VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x,y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

    } else if (sensor_type.compare("R") == 0) {
      // Skip Radar measurements
      continue;
    }
    ++i;
  }

  // Create a Tracking instance
  Tracking tracking;

  // call the ProcessingMeasurement() function for each measurement
  size_t N = measurement_pack_list.size();
  // start filtering from the second frame
  // (the speed is unknown in the first frame)
  for (size_t k = 0; k < N; ++k) {
    tracking.ProcessMeasurement(measurement_pack_list[k]);
  }

  if (in_file.is_open()) {
    in_file.close();
  }
  return 0;
}
