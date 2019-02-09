#include "StdAfx.h"
#include "lpNarrowPhase.h"



lpCollisionPrimitive::lpCollisionPrimitive()
{
	m_transform.identity();
}


lpCollisionPrimitive::~lpCollisionPrimitive()
{

}



lpCollisionGeometry::lpCollisionGeometry(void)
{
}


lpCollisionGeometry::~lpCollisionGeometry(void)
{
	m_geometries.~vector();
}


void lpCollisionGeometry::addGeometry(lpCollisionPrimitive *geom)
{
	if (geom)
	{
		m_geometries.push_back(geom);
	}
}


lpSphere::lpSphere(float radius) : lpCollisionPrimitive()
{
	
	m_type = PRIMITIVE_SPHERE;
	m_radius = radius;
}


float lpSphere::getVolume()
{
	return 4.0f / 3.0f * (float)M_PI * m_radius * m_radius * m_radius;
}



lpBox::lpBox(float width, float height, float depth) : lpCollisionPrimitive()
{
	m_type = PRIMITIVE_BOX;
	m_radius.setTo(0.5f * width, 0.5f * height, 0.5f * depth);
}


lpBox::lpBox(const lpVec3 &boxSize) : lpCollisionPrimitive()
{
	m_type = PRIMITIVE_BOX;
	m_radius = boxSize;
	m_radius.scale(0.5f);
}


float lpBox::getVolume()
{
	return 8 * m_radius.m_x * m_radius.m_y * m_radius.m_z;
}