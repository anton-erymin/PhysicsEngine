#include "StdAfx.h"
#include "lpInertia.h"


lpMat3 lpInertia::boxTensor(float mass, float width, float height, float depth)
{
	lpMat3 tensor;
	tensor.identity();

	tensor.m_data[0][0] = mass * (height * height + depth * depth) / 12.0f;
	tensor.m_data[1][1] = mass * (width * width + depth * depth) / 12.0f;
	tensor.m_data[2][2] = mass * (width * width + height * height) / 12.0f;

	return tensor;
}


lpMat3 lpInertia::sphereSolidTensor(float mass, float radius)
{
	lpMat3 tensor;
	tensor.identity();

	float I = 2.0f / 5.0f * mass * radius * radius;
	tensor.m_data[0][0] = I;
	tensor.m_data[1][1] = I;
	tensor.m_data[2][2] = I;

	return tensor;
}


lpMat3 lpInertia::sphereShellTensor(float mass, float radius)
{
	lpMat3 tensor;
	tensor.identity();

	float I = 2.0f / 3.0f * mass * radius * radius;
	tensor.m_data[0][0] = I;
	tensor.m_data[1][1] = I;
	tensor.m_data[2][2] = I;

	return tensor;
}