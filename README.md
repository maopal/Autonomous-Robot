# Sensor-FusionMadwick
Implementation of Sebastian Madwick's Sensor Fusion (Magnetomer, Acc and gyro sensors)

The Madgwick Sensor Fusion function (where the magic happens) is found here: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/ 


Madgwickupdate function provides orientation to a rigid body without Gimbal lock through the use of Quaternions, whilst being less computationally expensive when compared to the Kalman filter.

Kris Winer GitHub Page is a great for learning how to implement Madgwick sensor fusion: https://github.com/kriswiner

I have used the MPU9250 (sensors) & STM32F401RB (MCU):
 Step 1: Use i2c communication to continuously read  MAG, ACC & Gyro data.
 
 Step 2: Correct for bias in Gyro & Accel on start of program
 
 Step 3: Calibrate the Magnetometer Sensor - I did this step in Python  however Kriswiner did it within the program (see https://github.com/kriswiner)
 
 Step4: Input Mag, ACC & Gyro values in North East Down (NED) form in madwickupdate. (TAKE CARE OF MAG AXIS MISALIGNMENT WITH ACC & GYRO) - See MPU9250 Datasheet to check visual representation of MAG axis compared to ACC & GYRO.
 
 Step 5: Convert quaternions to Euleur Angles, and send data through UART
 
 Step 6: Monitor data in Python and produce live visualisation of rotation (see Visualisation.py)
