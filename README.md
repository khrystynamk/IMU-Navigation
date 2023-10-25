# Navigation using IMU

### Contributors

Mariia Ivanchenko,
Khrystyna Mysak, 
Dzvenyslava Butynets,
Ksenia Kretsula.

---

### IMU

**I**nertial **M**easurement **U**nit is a specific type of sensor that measures angular rate, force and sometimes magnetic field.

There are many different kinds of IMU devices each containing various measuring tools that pertain to specific uses. For example a 3-axis gyroscope and accelerometer would make up a 6-axis IMU while a 9-axis IMU also contains a 3-axis magnetometer.

**Accelerometers** are one of the most commonly used motion sensors and are used to measure the rate of change in an object's velocity due to linear forces acting upon it.

We can integrate a vehicle's acceleration to calculate velocity and integrate velocity to calculate distance traveled. So using accelerometer data it is possible to calculate distance from a set start point. In order to more accurately determine the position of a vehicle in 3d space **more data types are required**.

Acceleration data alone does not account for turns and non-linear movement that a vehicle makes along its journey. To solve this most IMU devices contain a **gyroscope** which measures angular velocity of three axes roll about the y-axis pitch about the x-axis and yaw about the z-axis. Although gyroscopes have no initial frame of reference like accelerometers when data from both is combined using **sensor fusion** techniques a vehicle's position and orientation can be reliably calculated. Also, the problem for gyroscope is the **zero drift** that has to be compensated for any usable application.

**Magnetometers** are used to measure the strength and direction of a magnetic field. In IMUs, they can provide a point of reference. A magnetometer is mainly used for speed detection and with fusion algorithm it can eliminate the gyroscope offset.

Since IMU does not use known reference points calculations can often contain small errors which in time can add up to larger errors. The **Kalman filtering** that uses only accelerometer and gyroscope, is a relatively simple state-space algorithm to produce estimates of the hidden variables based on uncertain and inaccurate measurements. It predicts the systems future state based on past estimations and is applied to model systems with multiple noisy inputs and outputs hopefully less noisy and more accurately estimated output data.

Other commonly known algorithms are Sebastian Madgwick's fusion algorithm and Mahony's algorithm.

---

### Kalman filter

The Kalman Filter is a generic algorithm that is used to estimate system parameters. It can use inaccurate or noisy measurements to estimate the state of that variable or another unobservable variable with greater accuracy.

---

### Madgwick filter

This is an orientation filter applicable to IMUs consisting of 3-axial gyroscopes and accelerometers, and MARG (magnetic angular rate and gravity) arrays, which also include 3-axial magnetometers, proposed by Sebastian Madgwick.

The filter employs a quaternion representation of orientation to describe the nature of orientations in three-dimensions and is not subject to the singularities associated with an Euler angle representation, allowing accelerometer and magnetometer data to be used in an analytically derived and optimised gradient-descent algorithm to compute the direction of the gyroscope measurement error as a quaternion derivative.

Innovative aspects of this filter include:

- A single adjustable parameter defined by observable systems characteristics.
- An analytically derived and optimised gradient-descent algorithm enabling performance at low sampling rates.
- On-line magnetic distortion compensation algorithm.
- Gyroscope bias drift compensation.

---

### Mahony filter

The Mahony filter is a glorified Complementary Filter with significant improvements to accuracy without significant markup in computation time. Even today, it remains to be one of the most popular filters used in racing quadrotors where time is money, only to be bettered by the Madgwick Filter with comparable computation time and slightly better accuracy.

The Mahony filter is a Complementary filter which respects the manifold transformations in **quaternion** space. The general idea of the Mahony filter is to estimate the attitude/angle/orientation by fusing/combining attitude estimates by integrating **gyro measurements** and **direction** obtained by the accelerometer measurements. An orientation error from previous step is first calculated to which a correction step based on a Proportional-Integral (PI) compensator is applied to correct the measured angular velocity. This angular velocity is propagated on the quaternion manifold and integrated to obtain the estimate of the attitude.

Algorithm can be divided into the following steps:

- **Step 1: Obtain sensor measurements**

Obtain and denote gyro and acc measurements from the sensor.  Also, denote the normalised acc measurements.

- **Step 2: Orientation error using Acc Measurements**

Compute orientation error from previous estimate using acc measurements.

- **Step 3: Update Gyro Measurements using PI Compensation (Fusion)**

Compute updated gyro measurements after the application of the PI compensation term.

- **Step 4: Orientation increment from Gyro**

Compute orientation increment from gyro measurements.

- **Step 5: Numerical Integration**

Compute orientation by integrating orientation increment.

- **Repeat steps 1 to 5 for every time instant.**

In a Mahony filter, the user needs to specify the initial estimates of the attitude, biases and sampling time. The initial attitude can be assumed to be zero if the device is at rest or it has to be obtained by external sources such as a motion capture system or a camera. The bias is computed by taking an average of samples with the IMU at rest and computing the mean value. Note that this bias changes over time and the filter will start to drift over time.

---

### Microcontroller

In this project we are using RISC Microcontroller: **STM32F411E-DISCO**.

---

### Components

- GY-521/MPU6050 — giro + acc
- FXOS8700+FXAS21002 — giro + acc + mag

---

### Resources

https://ahrs.readthedocs.io/en/latest/filters/madgwick.html

https://nitinjsanket.github.io/tutorials/attitudeest/mahony.html