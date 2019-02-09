#include "StdAfx.h"
#include "lpCollisionTest.h"




lpContactJoint* getFreeContact(std::vector<lpContactJoint*> *contacts, unsigned int numContacts)
{
	lpContactJoint *contact = 0;
	if (numContacts < contacts->size())
	{
		contact = (*contacts)[numContacts];
	}
	else
	{
		contact = new lpContactJoint();
		contacts->push_back(contact);
	}

	return contact;
}


void lpCollisions(lpCollisionGeometry *g1, lpCollisionGeometry *g2, std::vector<lpContactJoint*> *contacts, unsigned int &numContacts)
{
	for (size_t i = 0; i < g1->m_geometries.size(); i++)
	{
		for (size_t j = 0; j < g2->m_geometries.size(); j++)
		{
			testGeoms(g1->m_geometries[i], g2->m_geometries[j], contacts, numContacts);
		}
	}

}


void testGeoms(lpCollisionPrimitive *geom1, lpCollisionPrimitive *geom2, std::vector<lpContactJoint*> *contacts, unsigned int &numContacts)
{
	switch (geom1->m_type)
	{
	case PRIMITIVE_SPHERE:
		switch (geom2->m_type)
		{
		case PRIMITIVE_SPHERE:
			testSphereSphere((lpSphere*)geom1, (lpSphere*)geom2, contacts, numContacts);
			break;
		case PRIMITIVE_BOX:
			testBoxSphere((lpBox*)geom2, (lpSphere*)geom1, contacts, numContacts);
			break;
		}
		break;

	case PRIMITIVE_BOX:
		switch (geom2->m_type)
		{
		case PRIMITIVE_SPHERE:
			testBoxSphere((lpBox*)geom1, (lpSphere*)geom2, contacts, numContacts);
			break;
		case PRIMITIVE_BOX:
			testBoxBox((lpBox*)geom1, (lpBox*)geom2, contacts, numContacts);
			break;
		}
	}

}


void testSphereSphere(lpSphere *s1, lpSphere *s2, std::vector<lpContactJoint*> *contacts, unsigned int &numContacts)
{
	lpVec3 cc = s1->m_body->m_pos - s2->m_body->m_pos;
	float len = cc.normSq();
	float sr = s1->m_radius + s2->m_radius;
	if (len >= sr * sr) return;

	len = sqrtf(len);
	lpVec3 normal = cc * (1.0f / len);


	lpContactJoint *contact = getFreeContact(contacts, numContacts);

	contact->m_normal = normal;
	contact->m_point = s2->m_body->m_pos.addScaledR(cc, 0.5f);
	contact->m_depth = sr - len;

	completeContact(contact, s1, s2);
	numContacts++;
}

#include "../Opcode/Opcode.h"

void testBoxSphere(lpBox *b, lpSphere *s, std::vector<lpContactJoint*> *contacts, unsigned int &numContacts)
{
	IceMaths::Sphere *icesph = new IceMaths::Sphere(IceMaths::Point(), 10.0f);
	//printf("%f\n", icesph->mRadius);

	lpVec3 center = s->m_body->m_pos;
	center -= b->m_body->m_pos;
	center = b->m_body->m_orientation.multTransposed(center);

	if (fabsf(center.m_x) - s->m_radius > b->m_radius.m_x ||
		fabsf(center.m_y) - s->m_radius > b->m_radius.m_y ||
		fabsf(center.m_z) - s->m_radius > b->m_radius.m_z) return;

	lpVec3 closest;

	bool inside1 = false, inside2 = false, inside3 = false;

	// Ќаходим ближайшую к сфере точку бокса

	lpVec3 boxr = b->m_radius;

	if (center.m_x > boxr.m_x) closest.m_x = boxr.m_x;
	else if (center.m_x < -boxr.m_x) closest.m_x = -boxr.m_x;
	else 
	{
		closest.m_x = center.m_x;
		inside1 = true;
	}

	if (center.m_y > boxr.m_y) closest.m_y = boxr.m_y;
	else if (center.m_y < -boxr.m_y) closest.m_y = -boxr.m_y;
	else
	{
		closest.m_y = center.m_y;
		inside2 = true;
	}

	if (center.m_z > boxr.m_z) closest.m_z = boxr.m_z;
	else if (center.m_z < -boxr.m_z) closest.m_z = -boxr.m_z;
	else
	{
		closest.m_z = center.m_z;
		inside3 = true;
	}

	lpContactJoint *contact;

	if (inside1 && inside2 && inside3)
	{
		// ≈сли центр сферы полностью внутри бокса
		float shallowest = 1e+38f;
		float deep;
		int side;

		deep = boxr.m_x - center.m_x; if (deep < shallowest) { shallowest = deep; side = 0; }
		deep = boxr.m_x + center.m_x; if (deep < shallowest) { shallowest = deep; side = 1; }
		deep = boxr.m_y - center.m_y; if (deep < shallowest) { shallowest = deep; side = 2; }
		deep = boxr.m_y + center.m_y; if (deep < shallowest) { shallowest = deep; side = 3; }
		deep = boxr.m_z - center.m_z; if (deep < shallowest) { shallowest = deep; side = 4; }
		deep = boxr.m_z + center.m_z; if (deep < shallowest) { shallowest = deep; side = 5; }
		
		switch (side)
		{
		case 0:
			closest.m_x = boxr.m_x;
			break;
		case 1:
			closest.m_x = -boxr.m_x;
			break;
		case 2:
			closest.m_y = boxr.m_y;
			break;
		case 3:
			closest.m_y = -boxr.m_y;
			break;
		case 4:
			closest.m_z = boxr.m_z;
			break;
		case 5:
			closest.m_z = -boxr.m_z;
			break;
		}

		contact = getFreeContact(contacts, numContacts);

		closest *= b->m_body->m_orientation;
		closest += b->m_body->m_pos;

		contact->m_normal = closest - s->m_body->m_pos;
		contact->m_normal *= 1 / shallowest;
		contact->m_point = s->m_body->m_pos;
		contact->m_depth = shallowest + s->m_radius;
	}
	else
	{
		float len = (closest - center).normSq();
		if (len > s->m_radius * s->m_radius) return;

		closest *= b->m_body->m_orientation;
		closest += b->m_body->m_pos;



		contact = getFreeContact(contacts, numContacts);

		contact->m_normal = s->m_body->m_pos - closest;
		contact->m_normal.normalize();
		contact->m_point = closest;
		contact->m_depth = s->m_radius - sqrtf(len);
	}


	completeContact(contact, s, b);
	numContacts++;
}


void completeContact(lpContactJoint *c, lpCollisionPrimitive *geom1, lpCollisionPrimitive *geom2)
{
	c->m_body1 = geom1->m_body;
	c->m_body2 = geom2->m_body;

	c->m_friction = 0.5f * (c->m_body1->m_surface->m_friction + c->m_body2->m_surface->m_friction);
	c->m_restitution = 0.5f * (c->m_body1->m_surface->m_restitution + c->m_body2->m_surface->m_restitution);
}




int separationAxisIndex;
float smallestPenetration;

bool inline boxSAT(lpBox *b1, lpBox *b2, lpVec3 &axis, lpVec3 &betweenCenters, int index)
{
	float proj1 = b1->m_radius.m_x * fabsf(axis * b1->axis1) + b1->m_radius.m_y * fabsf(axis * b1->axis2) + b1->m_radius.m_z * fabsf(axis * b1->axis3);
	float proj2 = b2->m_radius.m_x * fabsf(axis * b2->axis1) + b2->m_radius.m_y * fabsf(axis * b2->axis2) + b2->m_radius.m_z * fabsf(axis * b2->axis3);

	float dist = fabsf(axis * betweenCenters);

	float penetration = proj1 + proj2 - dist;

	if (penetration >= 0.0f)
	{
		if (penetration < smallestPenetration)
		{
			smallestPenetration = penetration;
			separationAxisIndex = index;
		}
		return false;
	}

	return true;
}

void testBoxBox(lpBox *b1, lpBox *b2, std::vector<lpContactJoint*> *contacts, unsigned int &numContacts)
{
	printf("Box 2 Box Test: ");

	lpMat3 trans1 = b1->m_body->m_orientation;
	lpMat3 trans2 = b2->m_body->m_orientation;

	// Ќаходим главные оси боксов
	b1->axis1.setTo(trans1.m_data[0][0], trans1.m_data[1][0], trans1.m_data[2][0]);
	b1->axis2.setTo(trans1.m_data[0][1], trans1.m_data[1][1], trans1.m_data[2][1]);
	b1->axis3.setTo(trans1.m_data[0][2], trans1.m_data[1][2], trans1.m_data[2][2]);

	b2->axis1.setTo(trans2.m_data[0][0], trans2.m_data[1][0], trans2.m_data[2][0]);
	b2->axis2.setTo(trans2.m_data[0][1], trans2.m_data[1][1], trans2.m_data[2][1]);
	b2->axis3.setTo(trans2.m_data[0][2], trans2.m_data[1][2], trans2.m_data[2][2]);

	// Ќаходим вектор между центрами боксов
	lpVec3 betweenCenters = b2->m_body->m_pos - b1->m_body->m_pos;


	separationAxisIndex = -1;
	smallestPenetration = 10000000000.0f;

	// “еперь провер€ем 15 возможных раздел€ющих осей, при этом сохран€етс€ информации о наименьшей величине проникновени€

	/*if (boxSAT(b1, b2, b1->axis1, betweenCenters, 0)) return;
	if (boxSAT(b1, b2, b1->axis2, betweenCenters, 1)) return;
	if (boxSAT(b1, b2, b1->axis3, betweenCenters, 2)) return;

	if (boxSAT(b1, b2, b2->axis1, betweenCenters, 3)) return;
	if (boxSAT(b1, b2, b2->axis2, betweenCenters, 4)) return;
	if (boxSAT(b1, b2, b2->axis3, betweenCenters, 5)) return;

	if (boxSAT(b1, b2, b1->axis1 % b2->axis1, betweenCenters, 6)) return;
	if (boxSAT(b1, b2, b1->axis1 % b2->axis2, betweenCenters, 7)) return;
	if (boxSAT(b1, b2, b1->axis1 % b2->axis3, betweenCenters, 8)) return;
	if (boxSAT(b1, b2, b1->axis2 % b2->axis1, betweenCenters, 9)) return;
	if (boxSAT(b1, b2, b1->axis2 % b2->axis2, betweenCenters, 10)) return;
	if (boxSAT(b1, b2, b1->axis2 % b2->axis3, betweenCenters, 11)) return;
	if (boxSAT(b1, b2, b1->axis3 % b2->axis1, betweenCenters, 12)) return;
	if (boxSAT(b1, b2, b1->axis3 % b2->axis2, betweenCenters, 13)) return;
	if (boxSAT(b1, b2, b1->axis3 % b2->axis3, betweenCenters, 14)) return;*/

	printf("Penetrated\n");

	// “еперь мы знаем точно что есть коллизи€, и знаем кака€ ось дает наименьшую глубину проникновени€

	if (separationAxisIndex < 3)
	{
		// »меем контакты типа face-vertex
		// —толкнулись вершины второго бокса с гранью первого
	}
	else if (separationAxisIndex < 6)
	{
		// »меем контакты типа face-vertex
		// —толкнулись вершины первого бокса с гранью второго
	}
	else
	{
		
	}


	lpContactJoint *contact = getFreeContact(contacts, numContacts);

	contact->m_normal.setTo(0.0f, -1.0f, 0.0f);
	//contact->m_point = closest;
	contact->m_depth = smallestPenetration;

	completeContact(contact, b1, b2);
	numContacts++;
}