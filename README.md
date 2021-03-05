# Sensor-FusionMadwick
Implementation of Sebastian Madwick's Sensor Fusion (Magnetomer, Acc and gyro sensors)

The Madgwick Sensor Fusion function (where the magic happens) is found here: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Madgwickupdate function provides orientation to a rigid body without Gimbal lock through the use of Quaternions, whilst being less computationally expensive when comapred to the Kalman filter.
