#include "StdAfx.h"
#include "lpMath.h"


// lpVec3

lpVec3::lpVec3()
{
	clear();
}


lpVec3::lpVec3(float x, float y, float z)
{
	m_x = x;
	m_y = y;
	m_z = z;
}


lpVec3::lpVec3(float s)
{
	m_x = s;
	m_y = s;
	m_z = s;
}


lpVec3::lpVec3(float *v)
{
	m_x = v[0];
	m_y = v[1];
	m_z = v[2];
}


lpVec3::lpVec3(int axis)
{
	switch(axis)
	{
	case LP_X_AXIS:
		m_x = 1.0f; m_y = 0.0f; m_z = 0.0f;
		break;
	case LP_Y_AXIS:
		m_x = 0.0f; m_y = 1.0f; m_z = 0.0f;
		break;
	case LP_Z_AXIS:
		m_x = 0.0f; m_y = 0.0f; m_z = 1.0f;
		break;
	default:
		m_x = 0.0f; m_y = 0.0f; m_z = 0.0f;
	}
}


void lpVec3::clear()
{
	m_x = 0.0f;
	m_y = 0.0f;
	m_z = 0.0f;
}


void lpVec3::setTo(float x, float y, float z)
{
	m_x = x;
	m_y = y;
	m_z = z;
}


void lpVec3::add(const lpVec3& other)
{
	m_x += other.m_x;
	m_y += other.m_y;
	m_z += other.m_z;
}


lpVec3 lpVec3::addR(const lpVec3& other)
{
	return lpVec3(m_x + other.m_x, m_y + other.m_y, m_z + other.m_z);
}


void lpVec3::sub(const lpVec3& other)
{
	m_x -= other.m_x;
	m_y -= other.m_y;
	m_z -= other.m_z;
}


lpVec3 lpVec3::subR(const lpVec3& other)
{
	return lpVec3(m_x - other.m_x, m_y - other.m_y, m_z - other.m_z);
}


void lpVec3::scale(float s)
{
	m_x *= s;
	m_y *= s;
	m_z *= s;
}


lpVec3 lpVec3::scaleR(float s)
{
	return lpVec3(s * m_x, s * m_y, s * m_z);
}


void lpVec3::addScaled(const lpVec3& other, float s)
{
	m_x += s * other.m_x;
	m_y += s * other.m_y;
	m_z += s * other.m_z;
}


lpVec3 lpVec3::addScaledR(const lpVec3& other, float s)
{
	return lpVec3(m_x + s * other.m_x,	m_y + s * other.m_y, m_z + s * other.m_z);
}



float lpVec3::dot(const lpVec3& other)
{
	return m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;
}


void lpVec3::cross(const lpVec3& other)
{
	float nx = m_y * other.m_z - m_z * other.m_y;
	float ny = m_z * other.m_x - m_x * other.m_z;
	float nz = m_x * other.m_y - m_y * other.m_x;
	m_x = nx;
	m_y = ny;
	m_z = nz;
}


lpVec3 lpVec3::crossR(const lpVec3& other)
{
	return lpVec3(m_y * other.m_z - m_z * other.m_y, m_z * other.m_x - m_x * other.m_z, m_x * other.m_y - m_y * other.m_x);
}



float lpVec3::norm()
{
	return sqrtf(m_x * m_x + m_y * m_y + m_z * m_z);
}


float lpVec3::normSq()
{
	return m_x * m_x + m_y * m_y + m_z * m_z;
}


void lpVec3::normalize()
{
	float norm = sqrtf(m_x * m_x + m_y * m_y + m_z * m_z);

	if (norm > 0)
	{
		m_x /= norm;
		m_y /= norm;
		m_z /= norm;
	}
}


lpVec3 lpVec3::operator+(const lpVec3& other)
{
	return lpVec3(m_x + other.m_x, m_y + other.m_y, m_z + other.m_z);
}


void lpVec3::operator+=(const lpVec3& other)
{
	m_x += other.m_x;
	m_y += other.m_y;
	m_z += other.m_z;
}


lpVec3 lpVec3::operator-(const lpVec3& other)
{
	return lpVec3(m_x - other.m_x, m_y - other.m_y, m_z - other.m_z);
}


void lpVec3::operator -=(const lpVec3& other)
{
	m_x -= other.m_x;
	m_y -= other.m_y;
	m_z -= other.m_z;
}


float lpVec3::operator*(const lpVec3& other)
{
	return m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;
}


lpVec3 lpVec3::operator*(float s)
{
	return lpVec3(s * m_x, s * m_y, s * m_z);
}


void lpVec3::operator*=(float s)
{
	m_x *= s;
	m_y *= s;
	m_z *= s;
}


void lpVec3::operator*=(const lpMat3 &mat)
{
	float nx = mat.m_data[0][0] * m_x + mat.m_data[0][1] * m_y + mat.m_data[0][2] * m_z;
	float ny = mat.m_data[1][0] * m_x + mat.m_data[1][1] * m_y + mat.m_data[1][2] * m_z;
	float nz = mat.m_data[2][0] * m_x + mat.m_data[2][1] * m_y + mat.m_data[2][2] * m_z;
	m_x = nx;
	m_y = ny;
	m_z = nz;
}


lpVec3 lpVec3::operator %(const lpVec3& other)
{
	return lpVec3(m_y * other.m_z - m_z * other.m_y, m_z * other.m_x - m_x * other.m_z, m_x * other.m_y - m_y * other.m_x);
}


void lpVec3::operator %=(const lpVec3& other)
{
	float nx = m_y * other.m_z - m_z * other.m_y;
	float ny = m_z * other.m_x - m_x * other.m_z;
	float nz = m_x * other.m_y - m_y * other.m_x;
	m_x = nx;
	m_y = ny;
	m_z = nz;
}


// lpMat3

lpMat3::lpMat3() { }


lpMat3::lpMat3(float s)
{
	m_data[0][0] = s; m_data[0][1] = s; m_data[0][2] = s;
	m_data[1][0] = s; m_data[1][1] = s; m_data[1][2] = s;
	m_data[2][0] = s; m_data[2][1] = s; m_data[2][2] = s;
}


lpMat3::lpMat3(float **v)
{
	m_data[0][0] = v[0][0]; m_data[0][1] = v[0][1]; m_data[0][2] = v[0][2];
	m_data[1][0] = v[1][0]; m_data[1][1] = v[1][1]; m_data[1][2] = v[1][2];
	m_data[2][0] = v[2][0]; m_data[2][1] = v[2][1]; m_data[2][2] = v[2][2];
}


void lpMat3::clear()
{
	m_data[0][0] = 0.0f; m_data[0][1] = 0.0f; m_data[0][2] = 0.0f;
	m_data[1][0] = 0.0f; m_data[1][1] = 0.0f; m_data[1][2] = 0.0f;
	m_data[2][0] = 0.0f; m_data[2][1] = 0.0f; m_data[2][2] = 0.0f;
}


void lpMat3::identity()
{
	m_data[0][0] = 1.0f; m_data[0][1] = 0.0f; m_data[0][2] = 0.0f;
	m_data[1][0] = 0.0f; m_data[1][1] = 1.0f; m_data[1][2] = 0.0f;
	m_data[2][0] = 0.0f; m_data[2][1] = 0.0f; m_data[2][2] = 1.0f;
}


void lpMat3::add(const lpMat3 &other)
{
	m_data[0][0] += other.m_data[0][0]; m_data[0][1] += other.m_data[0][1]; m_data[0][2] += other.m_data[0][2];
	m_data[1][0] += other.m_data[1][0]; m_data[1][1] += other.m_data[1][1]; m_data[1][2] += other.m_data[1][2];
	m_data[2][0] += other.m_data[2][0]; m_data[2][1] += other.m_data[2][1]; m_data[2][2] += other.m_data[2][2];
}


lpMat3 lpMat3::addR(const lpMat3 &other)
{
	lpMat3 res;

	res.m_data[0][0] = m_data[0][0] + other.m_data[0][0];
	res.m_data[0][1] = m_data[0][1] + other.m_data[0][1];
	res.m_data[0][2] = m_data[0][2] + other.m_data[0][2];

	res.m_data[1][0] = m_data[1][0] + other.m_data[1][0];
	res.m_data[1][1] = m_data[1][1] + other.m_data[1][1];
	res.m_data[1][2] = m_data[1][2] + other.m_data[1][2];

	res.m_data[2][0] = m_data[2][0] + other.m_data[2][0];
	res.m_data[2][1] = m_data[2][1] + other.m_data[2][1];
	res.m_data[2][2] = m_data[2][2] + other.m_data[2][2];

	return res;
}


void lpMat3::sub(const lpMat3 &other)
{
	m_data[0][0] -= other.m_data[0][0]; m_data[0][1] -= other.m_data[0][1]; m_data[0][2] -= other.m_data[0][2];
	m_data[1][0] -= other.m_data[1][0]; m_data[1][1] -= other.m_data[1][1]; m_data[1][2] -= other.m_data[1][2];
	m_data[2][0] -= other.m_data[2][0]; m_data[2][1] -= other.m_data[2][1]; m_data[2][2] -= other.m_data[2][2];
}


lpMat3 lpMat3::subR(const lpMat3 &other)
{
	lpMat3 res;

	res.m_data[0][0] = m_data[0][0] - other.m_data[0][0];
	res.m_data[0][1] = m_data[0][1] - other.m_data[0][1];
	res.m_data[0][2] = m_data[0][2] - other.m_data[0][2];

	res.m_data[1][0] = m_data[1][0] - other.m_data[1][0];
	res.m_data[1][1] = m_data[1][1] - other.m_data[1][1];
	res.m_data[1][2] = m_data[1][2] - other.m_data[1][2];

	res.m_data[2][0] = m_data[2][0] - other.m_data[2][0];
	res.m_data[2][1] = m_data[2][1] - other.m_data[2][1];
	res.m_data[2][2] = m_data[2][2] - other.m_data[2][2];

	return res;
}


void lpMat3::scale(float s)
{
	m_data[0][0] *= s; m_data[0][1] *= s; m_data[0][2] *= s;
	m_data[1][0] *= s; m_data[1][1] *= s; m_data[1][2] *= s;
	m_data[2][0] *= s; m_data[2][1] *= s; m_data[2][2] *= s;
}


lpMat3 lpMat3::scaleR(float s)
{
	lpMat3 res;

	res.m_data[0][0] = s * m_data[0][0];
	res.m_data[0][1] = s * m_data[0][1];
	res.m_data[0][2] = s * m_data[0][2];

	res.m_data[1][0] = s * m_data[1][0];
	res.m_data[1][1] = s * m_data[1][1];
	res.m_data[1][2] = s * m_data[1][2];

	res.m_data[2][0] = s * m_data[2][0];
	res.m_data[2][1] = s * m_data[2][1];
	res.m_data[2][2] = s * m_data[2][2];

	return res;
}


void lpMat3::addScaled(const lpMat3 &other, float s)
{
	m_data[0][0] += s * other.m_data[0][0]; m_data[0][1] += s * other.m_data[0][1]; m_data[0][2] += s * other.m_data[0][2];
	m_data[1][0] += s * other.m_data[1][0]; m_data[1][1] += s * other.m_data[1][1]; m_data[1][2] += s * other.m_data[1][2];
	m_data[2][0] += s * other.m_data[2][0]; m_data[2][1] += s * other.m_data[2][1]; m_data[2][2] += s * other.m_data[2][2];
}


lpMat3 lpMat3::addScaledR(const lpMat3 &other, float s)
{
	lpMat3 res;

	res.m_data[0][0] = m_data[0][0] + s * other.m_data[0][0];
	res.m_data[0][1] = m_data[0][1] + s * other.m_data[0][1];
	res.m_data[0][2] = m_data[0][2] + s * other.m_data[0][2];

	res.m_data[1][0] = m_data[1][0] + s * other.m_data[1][0];
	res.m_data[1][1] = m_data[1][1] + s * other.m_data[1][1];
	res.m_data[1][2] = m_data[1][2] + s * other.m_data[1][2];

	res.m_data[2][0] = m_data[2][0] + s * other.m_data[2][0];
	res.m_data[2][1] = m_data[2][1] + s * other.m_data[2][1];
	res.m_data[2][2] = m_data[2][2] + s * other.m_data[2][2];

	return res;
}


void lpMat3::transpose()
{
	float temp;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			temp = m_data[i][j];
			m_data[i][j] = m_data[j][i];
			m_data[j][i] = temp;
		}
	}
}


lpVec3 lpMat3::multTransposed(const lpVec3 &vec)
{
	lpVec3 res;

	res.m_x = m_data[0][0] * vec.m_x + m_data[1][0] * vec.m_y + m_data[2][0] * vec.m_z;
	res.m_y = m_data[0][1] * vec.m_x + m_data[1][1] * vec.m_y + m_data[2][1] * vec.m_z;
	res.m_z = m_data[0][2] * vec.m_x + m_data[1][2] * vec.m_y + m_data[2][2] * vec.m_z;

	return res;
}


void lpMat3::multLeft(const lpMat3 &other)
{
	float m00 = other.m_data[0][0]*m_data[0][0] + other.m_data[0][1]*m_data[1][0] + other.m_data[0][2]*m_data[2][0];
	float m01 = other.m_data[0][0]*m_data[0][1] + other.m_data[0][1]*m_data[1][1] + other.m_data[0][2]*m_data[2][1];
	float m02 = other.m_data[0][0]*m_data[0][2] + other.m_data[0][1]*m_data[1][2] + other.m_data[0][2]*m_data[2][2];

	float m10 = other.m_data[1][0]*m_data[0][0] + other.m_data[1][1]*m_data[1][0] + other.m_data[1][2]*m_data[2][0];
	float m11 = other.m_data[1][0]*m_data[0][1] + other.m_data[1][1]*m_data[1][1] + other.m_data[1][2]*m_data[2][1];
	float m12 = other.m_data[1][0]*m_data[0][2] + other.m_data[1][1]*m_data[1][2] + other.m_data[1][2]*m_data[2][2];

	float m20 = other.m_data[2][0]*m_data[0][0] + other.m_data[2][1]*m_data[1][0] + other.m_data[2][2]*m_data[2][0];
	float m21 = other.m_data[2][0]*m_data[0][1] + other.m_data[2][1]*m_data[1][1] + other.m_data[2][2]*m_data[2][1];
	float m22 = other.m_data[2][0]*m_data[0][2] + other.m_data[2][1]*m_data[1][2] + other.m_data[2][2]*m_data[2][2];

	m_data[0][0] = m00; m_data[0][1] = m01; m_data[0][2] = m02;
	m_data[1][0] = m10; m_data[1][1] = m11; m_data[1][2] = m12;
	m_data[2][0] = m20; m_data[2][1] = m21; m_data[2][2] = m22;
}


void lpMat3::rotate(float angle, const lpVec3 &axis)
{
	float c = cosf(angle);
	float s = sinf(angle);
	float c1 = 1.0f - c;
	float u1 = axis.m_x;
	float u2 = axis.m_y;
	float u3 = axis.m_z;
	float u12 = u1 * u2;
	float u13 = u1 * u3;
	float u23 = u2 * u3;

	lpMat3 rm;
	rm.m_data[0][0] = c1 * u1 * u1 + c;	 rm.m_data[0][1] = c1 * u12 - s * u3; rm.m_data[0][2] = c1 * u13 + s * u2;
	rm.m_data[1][0] = c1 * u12 + s * u3; rm.m_data[1][1] = c1 * u2 * u2 + c;  rm.m_data[1][2] = c1 * u23 - s * u1;
	rm.m_data[2][0] = c1 * u13 - s * u2; rm.m_data[2][1] = c1 * u23 + s * u1; rm.m_data[2][2] = c1 * u3 * u3 + c;

	multLeft(rm);
}


lpMat3 lpMat3::operator+(const lpMat3 &other)
{
	lpMat3 res;

	res.m_data[0][0] = m_data[0][0] + other.m_data[0][0];
	res.m_data[0][1] = m_data[0][1] + other.m_data[0][1];
	res.m_data[0][2] = m_data[0][2] + other.m_data[0][2];

	res.m_data[1][0] = m_data[1][0] + other.m_data[1][0];
	res.m_data[1][1] = m_data[1][1] + other.m_data[1][1];
	res.m_data[1][2] = m_data[1][2] + other.m_data[1][2];

	res.m_data[2][0] = m_data[2][0] + other.m_data[2][0];
	res.m_data[2][1] = m_data[2][1] + other.m_data[2][1];
	res.m_data[2][2] = m_data[2][2] + other.m_data[2][2];

	return res;
}


void lpMat3::operator +=(const lpMat3 &other)
{
	m_data[0][0] += other.m_data[0][0]; m_data[0][1] += other.m_data[0][1]; m_data[0][2] += other.m_data[0][2];
	m_data[1][0] += other.m_data[1][0]; m_data[1][1] += other.m_data[1][1]; m_data[1][2] += other.m_data[1][2];
	m_data[2][0] += other.m_data[2][0]; m_data[2][1] += other.m_data[2][1]; m_data[2][2] += other.m_data[2][2];
}


lpMat3 lpMat3::operator-(const lpMat3 &other)
{
	lpMat3 res;

	res.m_data[0][0] = m_data[0][0] - other.m_data[0][0];
	res.m_data[0][1] = m_data[0][1] - other.m_data[0][1];
	res.m_data[0][2] = m_data[0][2] - other.m_data[0][2];

	res.m_data[1][0] = m_data[1][0] - other.m_data[1][0];
	res.m_data[1][1] = m_data[1][1] - other.m_data[1][1];
	res.m_data[1][2] = m_data[1][2] - other.m_data[1][2];

	res.m_data[2][0] = m_data[2][0] - other.m_data[2][0];
	res.m_data[2][1] = m_data[2][1] - other.m_data[2][1];
	res.m_data[2][2] = m_data[2][2] - other.m_data[2][2];

	return res;
}


void lpMat3::operator-=(const lpMat3 &other)
{
	m_data[0][0] -= other.m_data[0][0]; m_data[0][1] -= other.m_data[0][1]; m_data[0][2] -= other.m_data[0][2];
	m_data[1][0] -= other.m_data[1][0]; m_data[1][1] -= other.m_data[1][1]; m_data[1][2] -= other.m_data[1][2];
	m_data[2][0] -= other.m_data[2][0]; m_data[2][1] -= other.m_data[2][1]; m_data[2][2] -= other.m_data[2][2];
}


lpVec3 lpMat3::operator*(const lpVec3 &vec)
{
	lpVec3 res;

	res.m_x = m_data[0][0] * vec.m_x + m_data[0][1] * vec.m_y + m_data[0][2] * vec.m_z;
	res.m_y = m_data[1][0] * vec.m_x + m_data[1][1] * vec.m_y + m_data[1][2] * vec.m_z;
	res.m_z = m_data[2][0] * vec.m_x + m_data[2][1] * vec.m_y + m_data[2][2] * vec.m_z;

	return res;
}


lpMat3 lpMat3::operator*(const lpMat3 &other)
{
	lpMat3 res;

	res.m_data[0][0] = m_data[0][0]*other.m_data[0][0] + m_data[0][1]*other.m_data[1][0] + m_data[0][2]*other.m_data[2][0];
	res.m_data[0][1] = m_data[0][0]*other.m_data[0][1] + m_data[0][1]*other.m_data[1][1] + m_data[0][2]*other.m_data[2][1];
	res.m_data[0][2] = m_data[0][0]*other.m_data[0][2] + m_data[0][1]*other.m_data[1][2] + m_data[0][2]*other.m_data[2][2];

	res.m_data[1][0] = m_data[1][0]*other.m_data[0][0] + m_data[1][1]*other.m_data[1][0] + m_data[1][2]*other.m_data[2][0];
	res.m_data[1][1] = m_data[1][0]*other.m_data[0][1] + m_data[1][1]*other.m_data[1][1] + m_data[1][2]*other.m_data[2][1];
	res.m_data[1][2] = m_data[1][0]*other.m_data[0][2] + m_data[1][1]*other.m_data[1][2] + m_data[1][2]*other.m_data[2][2];

	res.m_data[2][0] = m_data[2][0]*other.m_data[0][0] + m_data[2][1]*other.m_data[1][0] + m_data[2][2]*other.m_data[2][0];
	res.m_data[2][1] = m_data[2][0]*other.m_data[0][1] + m_data[2][1]*other.m_data[1][1] + m_data[2][2]*other.m_data[2][1];
	res.m_data[2][2] = m_data[2][0]*other.m_data[0][2] + m_data[2][1]*other.m_data[1][2] + m_data[2][2]*other.m_data[2][2];

	return res;
}


void lpMat3::operator *=(const lpMat3 &other)
{
	float m00 = m_data[0][0]*other.m_data[0][0] + m_data[0][1]*other.m_data[1][0] + m_data[0][2]*other.m_data[2][0];
	float m01 = m_data[0][0]*other.m_data[0][1] + m_data[0][1]*other.m_data[1][1] + m_data[0][2]*other.m_data[2][1];
	float m02 = m_data[0][0]*other.m_data[0][2] + m_data[0][1]*other.m_data[1][2] + m_data[0][2]*other.m_data[2][2];

	float m10 = m_data[1][0]*other.m_data[0][0] + m_data[1][1]*other.m_data[1][0] + m_data[1][2]*other.m_data[2][0];
	float m11 = m_data[1][0]*other.m_data[0][1] + m_data[1][1]*other.m_data[1][1] + m_data[1][2]*other.m_data[2][1];
	float m12 = m_data[1][0]*other.m_data[0][2] + m_data[1][1]*other.m_data[1][2] + m_data[1][2]*other.m_data[2][2];

	float m20 = m_data[2][0]*other.m_data[0][0] + m_data[2][1]*other.m_data[1][0] + m_data[2][2]*other.m_data[2][0];
	float m21 = m_data[2][0]*other.m_data[0][1] + m_data[2][1]*other.m_data[1][1] + m_data[2][2]*other.m_data[2][1];
	float m22 = m_data[2][0]*other.m_data[0][2] + m_data[2][1]*other.m_data[1][2] + m_data[2][2]*other.m_data[2][2];

	m_data[0][0] = m00; m_data[0][1] = m01; m_data[0][2] = m02;
	m_data[1][0] = m10; m_data[1][1] = m11; m_data[1][2] = m12;
	m_data[2][0] = m20; m_data[2][1] = m21; m_data[2][2] = m22;
}



lpMat3 lpMat3::operator~()
{
	lpMat3 res;

	res.m_data[0][0] = m_data[0][0]; res.m_data[0][1] = m_data[1][0]; res.m_data[0][2] = m_data[2][0];
	res.m_data[1][0] = m_data[0][1]; res.m_data[1][1] = m_data[1][1]; res.m_data[1][2] = m_data[2][1];
	res.m_data[2][0] = m_data[0][2]; res.m_data[2][1] = m_data[1][2]; res.m_data[2][2] = m_data[2][2];

	return res;
}


void lpMat3::invert()
{
	float det = m_data[0][0] * m_data[1][1] * m_data[2][2] - m_data[0][0] * m_data[1][2] * m_data[2][1];
	det -= m_data[0][1] * m_data[1][0] * m_data[2][2] + m_data[0][1] * m_data[1][2] * m_data[2][0];
	det += m_data[0][2] * m_data[1][0] * m_data[2][1] - m_data[0][2] * m_data[1][1] * m_data[2][0];

	if (det != 0.0f)
	{
		det = 1.0f / det;

		float m00 =	  m_data[1][1] * m_data[2][2] - m_data[2][1] * m_data[1][2];
		float m01 = -(m_data[0][1] * m_data[2][2] - m_data[2][1] * m_data[0][2]);
		float m02 =   m_data[0][1] * m_data[1][2] - m_data[1][1] * m_data[0][2];

		float m10 =	-(m_data[1][0] * m_data[2][2] - m_data[2][0] * m_data[1][2]);
		float m11 =   m_data[0][0] * m_data[2][2] - m_data[2][0] * m_data[0][2];
		float m12 = -(m_data[0][0] * m_data[1][2] - m_data[1][0] * m_data[0][2]);

		float m20 =	  m_data[1][0] * m_data[2][1] - m_data[2][0] * m_data[1][1];
		float m21 = -(m_data[0][0] * m_data[2][1] - m_data[2][0] * m_data[0][1]);
		float m22 =   m_data[0][0] * m_data[1][1] - m_data[1][0] * m_data[0][1];

		m_data[0][0] = det * m00; m_data[0][1] = det * m01; m_data[0][2] = det * m02;
		m_data[1][0] = det * m10; m_data[1][1] = det * m11; m_data[1][2] = det * m12;
		m_data[2][0] = det * m20; m_data[2][1] = det * m21; m_data[2][2] = det * m22;
	}
}