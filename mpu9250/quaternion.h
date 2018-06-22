#define PI 3.14159265358979323846

bool madgwickQuaternionUpdate(
	double quaternion[4],
	double acceleration[3],
	double rotationRate[3],
	double magneticField[3],
	double deltat)
{
	double beta = 2.5;//sqrt(0.75)*PI/3.0;
	double q1 = quaternion[0], q2 = quaternion[1], q3 = quaternion[2], q4 = quaternion[3];
	double ax = acceleration[0], ay = acceleration[1], az = acceleration[2];
	double gx = (PI/180.0)*rotationRate[0], gy = (PI/180.0)*rotationRate[1], gz = (PI/180.0)*rotationRate[2];
	double mx = magneticField[0], my = magneticField[1], mz = magneticField[2];

	double _2q1 = 2.0 * q1;
	double _2q2 = 2.0 * q2;
	double _2q3 = 2.0 * q3;
	double _2q4 = 2.0 * q4;
	double _2q1q3 = 2.0 * q1 * q3;
	double _2q3q4 = 2.0 * q3 * q4;
	double q1q1 = q1 * q1;
	double q1q2 = q1 * q2;
	double q1q3 = q1 * q3;
	double q1q4 = q1 * q4;
	double q2q2 = q2 * q2;
	double q2q3 = q2 * q3;
	double q2q4 = q2 * q4;
	double q3q3 = q3 * q3;
	double q3q4 = q3 * q4;
	double q4q4 = q4 * q4;

	double norm = sqrt(ax * ax + ay * ay + az * az);
	if (abs(norm) < 1e-6) {
		return false;
	}
	norm = 1.0/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	norm = sqrt(mx * mx + my * my + mz * mz);
	if (abs(norm) < 1e-6) {
		return false;
	}
	norm = 1.0/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	double _2q1mx = 2.0 * q1 * mx;
	double _2q1my = 2.0 * q1 * my;
	double _2q1mz = 2.0 * q1 * mz;
	double _2q2mx = 2.0 * q2 * mx;
	double hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	double hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	double _2bx = sqrt(hx * hx + hy * hy);
	double _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	double _4bx = 2.0f * _2bx;
	double _4bz = 2.0f * _2bz;

	double s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	double s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	double s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	double s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); 
	norm = 1.0/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	double qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	double qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	double qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	double qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); 
	if (abs(norm) < 1e-6) {
		return false;
	}
	norm = 1.0/norm;
	quaternion[0] = q1 * norm;
	quaternion[1] = q2 * norm;
	quaternion[2] = q3 * norm;
	quaternion[3] = q4 * norm;
	return true;
}


