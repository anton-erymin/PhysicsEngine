#include "StdAfx.h"
#include "lpBroadPhase.h"


lpAxisAlignedBoundingBox::lpAxisAlignedBoundingBox(void)
{
}


lpAxisAlignedBoundingBox::lpAxisAlignedBoundingBox(const lpVec3 &radius)
{
	m_radius = radius;
}


lpAxisAlignedBoundingBox::~lpAxisAlignedBoundingBox(void)
{
}


bool lpAxisAlignedBoundingBox::overlapsAABB(const lpAxisAlignedBoundingBox &other)
{
	if (fabsf(m_center.m_x - other.m_center.m_x) >= m_radius.m_x + other.m_radius.m_x) return false;
	if (fabsf(m_center.m_y - other.m_center.m_y) >= m_radius.m_y + other.m_radius.m_y) return false;
	if (fabsf(m_center.m_z - other.m_center.m_z) >= m_radius.m_z + other.m_radius.m_z) return false;
	return true;
}


void lpAxisAlignedBoundingBox::transformAABB(const lpAxisAlignedBoundingBox &aabb, lpAxisAlignedBoundingBox &newAabb, const lpMat3 &transform, const lpVec3 &translation)
{
	int row = 0;

	newAabb.m_center.m_x = translation.m_x;
	newAabb.m_radius.m_x = 0.0f;
	newAabb.m_center.m_x += transform.m_data[row][0] * aabb.m_center.m_x + 
							transform.m_data[row][1] * aabb.m_center.m_y + 
							transform.m_data[row][2] * aabb.m_center.m_z;
	newAabb.m_radius.m_x = fabsf(transform.m_data[row][0]) * aabb.m_radius.m_x + 
						   fabsf(transform.m_data[row][1]) * aabb.m_radius.m_y +
						   fabsf(transform.m_data[row][2]) * aabb.m_radius.m_z;

	row = 1;
	newAabb.m_center.m_y = translation.m_y;
	newAabb.m_radius.m_y = 0.0f;
	newAabb.m_center.m_y += transform.m_data[row][0] * aabb.m_center.m_x + 
							transform.m_data[row][1] * aabb.m_center.m_y + 
							transform.m_data[row][2] * aabb.m_center.m_z;
	newAabb.m_radius.m_y = fabsf(transform.m_data[row][0]) * aabb.m_radius.m_x + 
						   fabsf(transform.m_data[row][1]) * aabb.m_radius.m_y +
						   fabsf(transform.m_data[row][2]) * aabb.m_radius.m_z;

	row = 2;
	newAabb.m_center.m_z = translation.m_z;
	newAabb.m_radius.m_z = 0.0f;
	newAabb.m_center.m_z += transform.m_data[row][0] * aabb.m_center.m_x + 
							transform.m_data[row][1] * aabb.m_center.m_y + 
							transform.m_data[row][2] * aabb.m_center.m_z;
	newAabb.m_radius.m_z = fabsf(transform.m_data[row][0]) * aabb.m_radius.m_x + 
						   fabsf(transform.m_data[row][1]) * aabb.m_radius.m_y +
						   fabsf(transform.m_data[row][2]) * aabb.m_radius.m_z;

}