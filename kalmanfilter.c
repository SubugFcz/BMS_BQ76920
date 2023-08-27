/*******************************************************************************
 * @file        kalmanfilter.c
 * @brief       Kalman Filter Library for SoC and SoH calculation
 * @version     1.0
 * @author      Ignatius Djaynurdin
 * @date        2023-7-26
 ********************************************************************************

 MIT License
 Copyright (c) 2018 ETA Systems
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#include "kalmanfilter.h"

void kalman_filter_init(KalmanFilter *kf, float initial_state_estimate, float initial_estimate_error_cov, float measurement_noise_cov) {
    // Initialize Kalman Filter parameters
    kf->x = initial_state_estimate;
    kf->P = initial_estimate_error_cov;
	kf->Q = 0.05f; // Process noise covariance (tune this based on your system)
    kf->R = measurement_noise_cov;  // Measurement noise covariance (varying based on sensor errors)
}

float kalman_filter_update(KalmanFilter *kf, float z, float measurement_noise_cov) {
    // Update step
    // Update measurement noise covariance R based on the provided value
    kf->R = measurement_noise_cov;

    // Kalman Gain calculation
    float K = kf->P / (kf->P + kf->R);

    // Update state estimate and estimate error covariance
    kf->x = kf->x + K * (z - kf->x);
    kf->P = (1 - K) * kf->P;

    return kf->x;
}
