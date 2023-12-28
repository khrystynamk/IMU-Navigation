"""
Madgwick filter implementation.
"""
import math


class Madgwick:
    """
    Madgwick filter.
    """
    def __init__(self):
        # It's a tuning parameter that determines the trade-off between trusting the gyroscope
        # measurements and trusting the accelerometer/magnetometer measurements.
        # A higher beta value implies greater trust in the gyroscope measurements. However,
        # too much trust in the gyroscope can lead to drift over time. The actual choice of
        # beta often involves experimentation and adjustment based on the specific characteristics
        # of the sensor and the application.
        self.beta = 0.1
        # Quaternion [q0, q1, q2, q3].
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        # From datasheet Sampling rate is 512Hz
        # (https://www.st.com/resource/en/datasheet/stm32f411ce.pdf, page 25).
        self.sampleFreqDef = 512.0
        self.invSampleFreq = 1.0 / self.sampleFreqDef
        # State 0 or 1 depending on whether we calculated roll, pitch and yaw.
        self.anglesComputed = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        """
        Update quaternion by receiving new measurements.
        """
        # Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533
        gy *= 0.0174533
        gz *= 0.0174533

        # Rate of change of quaternion from gyroscope.
        # Formulas: https://ahrs.readthedocs.io/en/latest/filters/angular.html.
        q_dot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        q_dot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        q_dot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        q_dot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        # Compute feedback only if accelerometer measurement valid
        # (avoids 0 in accelerometer normalisation).
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):

            # Normalise accelerometer measurement:
            # The purpose of the normalization is to ensure that the accelerometer readings,
            # when considered as a vector, have a unit magnitude.
            # sqrt(x^2 + y^2 + z^2) = 1
            recip_norm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

            # Normalise magnetometer measurement
            recip_norm = 1.0 / math.sqrt(mx * mx + my * my + mz * mz)
            mx *= recip_norm
            my *= recip_norm
            mz *= recip_norm

            # Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0 * self.q0 * mx
            _2q0my = 2.0 * self.q0 * my
            _2q0mz = 2.0 * self.q0 * mz
            _2q1mx = 2.0 * self.q1 * mx
            _2q0 = 2.0 * self.q0
            _2q1 = 2.0 * self.q1
            _2q2 = 2.0 * self.q2
            _2q3 = 2.0 * self.q3
            _2q0q2 = 2.0 * self.q0 * self.q2
            _2q2q3 = 2.0 * self.q2 * self.q3
            q0q0 = self.q0 * self.q0
            q0q1 = self.q0 * self.q1
            q0q2 = self.q0 * self.q2
            q0q3 = self.q0 * self.q3
            q1q1 = self.q1 * self.q1
            q1q2 = self.q1 * self.q2
            q1q3 = self.q1 * self.q3
            q2q2 = self.q2 * self.q2
            q2q3 = self.q2 * self.q3
            q3q3 = self.q3 * self.q3

            # Reference direction of Earth's magnetic field
            hx = (mx * q0q0 - _2q0my * self.q3 + _2q0mz * self.q2 + mx * q1q1 +
                  _2q1 * my * self.q2 + _2q1 * mz * self.q3 - mx * q2q2 - mx * q3q3)
            hy = (_2q0mx * self.q3 + my * q0q0 - _2q0mz * self.q1 + _2q1mx * self.q2 -
                  my * q1q1 + my * q2q2 + _2q2 * mz * self.q3 - my * q3q3)
            _2bx = math.sqrt(hx * hx + hy * hy)
            _2bz = (-_2q0mx * self.q2 + _2q0my * self.q1 + mz * q0q0 + _2q1mx * self.q3 -
                    mz * q1q1 + _2q2 * my * self.q3 - mz * q2q2 + mz * q3q3)

            _4bx = 2.0 * _2bx
            _4bz = 2.0 * _2bz

            s0 = (-_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) -
                  _2bz * self.q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                  (-_2bx * self.q3 + _2bz * self.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                  _2bx * self.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            s1 = (_2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) -
                  4.0 * self.q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) +
                  _2bz * self.q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                  (_2bx * self.q2 + _2bz * self.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                  (_2bx * self.q3 - _4bz * self.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            s2 = (-_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) -
                  4.0 * self.q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) +
                  (-_4bx * self.q2 - _2bz * self.q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                  (_2bx * self.q1 + _2bz * self.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                  (_2bx * self.q0 - _4bz * self.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            s3 = (_2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) +
                  (-_4bx * self.q3 + _2bz * self.q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                  (-_2bx * self.q0 + _2bz * self.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                  _2bx * self.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))

            recip_norm = 1.0 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            s0 *= recip_norm
            s1 *= recip_norm
            s2 *= recip_norm
            s3 *= recip_norm

            q_dot1 -= self.beta * s0
            q_dot2 -= self.beta * s1
            q_dot3 -= self.beta * s2
            q_dot4 -= self.beta * s3

        # Integrate rate of change of quaternion to yield quaternion
        self.q0 += q_dot1 * self.invSampleFreq
        self.q1 += q_dot2 * self.invSampleFreq
        self.q2 += q_dot3 * self.invSampleFreq
        self.q3 += q_dot4 * self.invSampleFreq

        # Normalise quaternion
        recip_norm = 1.0 / math.sqrt(self.q0 * self.q0 + self.q1 * self.q1 +
                                     self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= recip_norm
        self.q1 *= recip_norm
        self.q2 *= recip_norm
        self.q3 *= recip_norm

        self.anglesComputed = 0

    def compute_angles(self):
        """
        Roll, pitch and yaw.
        """
        self.roll = math.atan2(self.q0 * self.q1 + self.q2 * self.q3,
                               0.5 - self.q1 * self.q1 - self.q2 * self.q2)
        self.pitch = math.asin(-2.0 * (self.q1 * self.q3 - self.q0 * self.q2))
        self.yaw = math.atan2(self.q1 * self.q2 + self.q0 * self.q3,
                              0.5 - self.q2 * self.q2 - self.q3 * self.q3)
        self.anglesComputed = 1
