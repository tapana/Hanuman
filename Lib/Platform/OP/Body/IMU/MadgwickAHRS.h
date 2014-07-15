#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <math.h>

// MadgwickAHRS class. Implementation of Madgwick's IMU and AHRS algorithms.

class MadgwickAHRS
{
private :


public :
    float samplePeriod;

    //algorithm gain beta.
    float beta;

    //quaternion output.
    float quaternion[4];
    float rotationMatrix[9];

    // Initializes a new instance of the <see cref="MadgwickAHRS"/> class.


    void setParam(float _samplePeriod, float _beta=1.0f)
    {
        samplePeriod = _samplePeriod;
        beta = _beta;
        quaternion[0] = 1.0;
    }

    void calRotationMatrix(){
        float* r = rotationMatrix;
        float* q = quaternion;

        r[0] = 1- 2*q[2]*q[2] - 2*q[3]*q[3];
        r[1] = 2*q[1]*q[2] - 2*q[0]*q[3];
        r[2] = 2*q[1]*q[3] + 2*q[0]*q[2];

        r[3] = 2*q[1]*q[2] + 2*q[0]*q[3];
        r[4] = 1 - 2*q[1]*q[1] - 2*q[3]*q[3];
        r[5] = 2*q[2]*q[3] - 2*q[0]*q[1];

        r[6] = 2*q[1]*q[3] - 2*q[0]*q[2];
        r[7] = 2*q[2]*q[3] + 2*q[0]*q[1];
        r[8] = 1 - 2*q[1]*q[1] - 2*q[2]*q[2];


    }

    /// <summary>
    /// Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
    /// </summary>
    /// <param name="gx">
    /// Gyroscope x axis measurement in radians/s.
    /// </param>
    /// <param name="gy">
    /// Gyroscope y axis measurement in radians/s.
    /// </param>
    /// <param name="gz">
    /// Gyroscope z axis measurement in radians/s.
    /// </param>
    /// <param name="ax">
    /// Accelerometer x axis measurement in any calibrated units.
    /// </param>
    /// <param name="ay">
    /// Accelerometer y axis measurement in any calibrated units.
    /// </param>
    /// <param name="az">
    /// Accelerometer z axis measurement in any calibrated units.
    /// </param>
    /// <param name="mx">
    /// Magnetometer x axis measurement in any calibrated units.
    /// </param>
    /// <param name="my">
    /// Magnetometer y axis measurement in any calibrated units.
    /// </param>
    /// <param name="mz">
    /// Magnetometer z axis measurement in any calibrated units.
    /// </param>
    /// <remarks>
    /// Optimised for minimal arithmetic.
    /// Total ±: 160
    /// Total *: 172
    /// Total /: 5
    /// Total sqrt: 5
    /// </remarks>
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        float q1 = quaternion[0], q2 = quaternion[1], q3 = quaternion[2], q4 = quaternion[3];   // short name local variable for readability
        float norm;
        float hx, hy, _2bx, _2bz;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1mx;
        float _2q1my;
        float _2q1mz;
        float _2q2mx;
        float _4bx;
        float _4bz;
        float _2q1 = 2.f * q1;
        float _2q2 = 2.f * q2;
        float _2q3 = 2.f * q3;
        float _2q4 = 2.f * q4;
        float _2q1q3 = 2.f * q1 * q3;
        float _2q3q4 = 2.f * q3 * q4;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = (float)sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = (float)sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2.f * q1 * mx;
        _2q1my = 2.f * q1 * my;
        _2q1mz = 2.f * q1 * mz;
        _2q2mx = 2.f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = (float)sqrt(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2.f * _2bx;
        _4bz = 2.f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.f * q2q4 - _2q1q3 - ax) + _2q2 * (2.f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.f * q2q4 - _2q1q3 - ax) + _2q1 * (2.f * q1q2 + _2q3q4 - ay) - 4.f * q2 * (1 - 2.f * q2q2 - 2.f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.f * q2q4 - _2q1q3 - ax) + _2q4 * (2.f * q1q2 + _2q3q4 - ay) - 4.f * q3 * (1 - 2.f * q2q2 - 2.f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.f * q2q4 - _2q1q3 - ax) + _2q3 * (2.f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = 1.f / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * samplePeriod;
        q2 += qDot2 * samplePeriod;
        q3 += qDot3 * samplePeriod;
        q4 += qDot4 * samplePeriod;
        norm = 1.f / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        quaternion[0] = q1 * norm;
        quaternion[1] = q2 * norm;
        quaternion[2] = q3 * norm;
        quaternion[3] = q4 * norm;
    }

    /// <summary>
    /// Algorithm IMU update method. Requires only gyroscope and accelerometer data.
    /// </summary>
    /// <param name="gx">
    /// Gyroscope x axis measurement in radians/s.
    /// </param>
    /// <param name="gy">
    /// Gyroscope y axis measurement in radians/s.
    /// </param>
    /// <param name="gz">
    /// Gyroscope z axis measurement in radians/s.
    /// </param>
    /// <param name="ax">
    /// Accelerometer x axis measurement in any calibrated units.
    /// </param>
    /// <param name="ay">
    /// Accelerometer y axis measurement in any calibrated units.
    /// </param>
    /// <param name="az">
    /// Accelerometer z axis measurement in any calibrated units.
    /// </param>
    /// <remarks>
    /// Optimised for minimal arithmetic.
    /// Total ±: 45
    /// Total *: 85
    /// Total /: 3
    /// Total sqrt: 3
    /// </remarks>
     void update(float gx, float gy, float gz, float ax, float ay, float az)
    {
        float q1 = quaternion[0], q2 = quaternion[1], q3 = quaternion[2], q4 = quaternion[3];   // short name local variable for readability
        float norm;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1 = 2.f * q1;
        float _2q2 = 2.f * q2;
        float _2q3 = 2.f * q3;
        float _2q4 = 2.f * q4;
        float _4q1 = 4.f * q1;
        float _4q2 = 4.f * q2;
        float _4q3 = 4.f * q3;
        float _8q2 = 8.f * q2;
        float _8q3 = 8.f * q3;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = (float)sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
        s2 = _4q2 * q4q4 - _2q4 * ax + 4.f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
        s3 = 4.f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
        s4 = 4.f * q2q2 * q4 - _2q2 * ax + 4.f * q3q3 * q4 - _2q3 * ay;
        norm = 1.f / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * samplePeriod;
        q2 += qDot2 * samplePeriod;
        q3 += qDot3 * samplePeriod;
        q4 += qDot4 * samplePeriod;
        norm = 1.f / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        quaternion[0] = q1 * norm;
        quaternion[1] = q2 * norm;
        quaternion[2] = q3 * norm;
        quaternion[3] = q4 * norm;
    }

};

#endif
