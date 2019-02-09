#include "StdAfx.h"
#include "lpLaxePhysicsEngine.h"




lpLaxePhysicsEngine::lpLaxePhysicsEngine()
{
	m_world = (lpWorld*)new lpWorldSimple(SOLVER_LCP);
}


lpLaxePhysicsEngine::~lpLaxePhysicsEngine()
{
}


void lpLaxePhysicsEngine::setGravity(const lpVec3 &gravity)
{
	if (m_world)
	{
		m_world->setGravity(gravity);
	}
}


void lpLaxePhysicsEngine::step(float dt)
{
	m_world->step(dt);
}


void lpLaxePhysicsEngine::addBody(lpRigidBody *body)
{
	if (m_world)
	{
		m_world->addBody(body);
	}
}


void lpLaxePhysicsEngine::removeBody(const lpRigidBody *body)
{
	if (m_world)
	{
		m_world->removeBody(body);
	}
}


void lpLaxePhysicsEngine::setWorld(lpWorld *world)
{
	if (world)
	{
		m_world = world;
	}
}


lpWorld* lpLaxePhysicsEngine::getWorld()
{
	return m_world;
}


lpRigidBody* lpLaxePhysicsEngine::createRigidBody(bool addFlag)
{
	lpRigidBody *body = new lpRigidBody();
	if (addFlag) addBody(body);
	return body;
}


lpRigidBody* lpLaxePhysicsEngine::createStaticBody(bool addFlag)
{
	lpRigidBody *body = new lpRigidBody();
	body->makeImmovable();
	if (addFlag) addBody(body);
	return body;
}


lpWorld* lpLaxePhysicsEngine::createWorldSimple(int solverType)
{
	return new lpWorldSimple(solverType);
}


lpWorld* lpLaxePhysicsEngine::createWorldQuadtree(int solverType)
{
	return 0;
}


lpWorld* lpLaxePhysicsEngine::createWorldBVH(int solverType)
{
	return 0;
}