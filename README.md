# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

# Rubric Points


## Result:

The result is generated based on the given sample data `obj_pose-laser-radar-synthetic-input.txt`.

For data set 1:

| `Data Set` | `RMSE X` | `RMSE Y` | `RMSE vx` | `RMSE vy` |
|------------|----------|----------|-----------|-----------|
| `1`        | `0.0686` | `0.0814` | `0.3291`  | `0.1890`  |

For data set 2:

| `Data Set` | `RMSE X` | `RMSE Y` | `RMSE vx` | `RMSE vy` |
|------------|----------|----------|-----------|-----------|
| `2`        | `0.0682` | `0.0688` | `0.3691`  | `0.2060`  |

## Algorithms:

The general process flow can be found in the function `ProcessMeasurement` from the file `ukf.cpp`

The function handles first measurement as following:

<pre><code>if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        double rho = meas_package.raw_measurements_[0]; // range
        double phi = meas_package.raw_measurements_[1]; // angular distance
        double rho_dot = meas_package.raw_measurements_[2]; // rate of change of rho
        double px = rho * cos(phi);
        double py = rho * sin(phi);
        double vx = rho_dot * cos(phi);
        double vy = rho_dot * sin(phi);
        double v = sqrt(vx * vx + vy * vy);
        x_ << px, py, 0, 0, 0;
        // x_ << px, py, v, phi, 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        double px = meas_package.raw_measurements_[0];
        double py = meas_package.raw_measurements_[1];
        x_ << px, py, 0, 0, 0;
    }

    P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 10, 0, 0,
            0, 0, 0, 50, 0,
            0, 0, 0, 0, 3;

    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
}
</code></pre>

Then it first predict then updates as follows:

<pre><code>
    // Predict
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    Prediction(delta_t);

    // Update
    if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_)) {
        UpdateRadar(meas_package);
    } else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && (use_laser_)) {
        UpdateLidar(meas_package);
    }
</code></pre>

For specific implementation of lidar and radar update, please refer to the code in `ukf.cpp` at line 172 and 205 respectively. 