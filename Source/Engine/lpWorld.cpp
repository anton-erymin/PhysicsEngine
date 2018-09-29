#include "StdAfx.h"
#include "lpWorld.h"



lpWorld::lpWorld(int solverType)
{
	// Создаем нужный солвер
	if (solverType == SOLVER_SEQUENTIAL)
	{
		// Солвер на основе последовательных импульсов
		m_solver = (lpSolver*)new lpSequentialSolver();
	}
	else if (solverType == SOLVER_LCP)
	{
		// Солвер на основе решения LCP	
		m_solver = (lpSolver*)new lpLCPSolver();
	}

	m_numDynamicBodies = 0;
}

lpWorld::~lpWorld()
{
}


void lpWorld::addBody(lpRigidBody *body)
{
	if (body)
	{
		body->setGravity(m_gravity);
		m_bodies.push_back((lpRigidBody*)body);
		if (!body->m_immovable)
			m_numDynamicBodies++;
	}
}


void lpWorld::removeBody(const lpRigidBody *body)
{
	if (body)
	{
		for (size_t i = 0; i < m_bodies.size(); i++)
		{
			if (m_bodies[i] == body)
			{
				if (!body->m_immovable)
					m_numDynamicBodies--;
				
				lpRigidBody *temp = m_bodies[m_bodies.size() - 1];
				m_bodies.pop_back();
				m_bodies[i] = temp;
				return;
			}
		}
	}
}


void lpWorld::setGravity(const lpVec3 &gravity)
{
	m_gravity = gravity;
	for (size_t i = 0; i < m_bodies.size(); i++)
	{
		m_bodies[i]->setGravity(m_gravity);
	}
}


void lpWorld::step(float dt)
{
	for (size_t i = 0; i < m_bodies.size(); i++)
	{
		if (!m_bodies[i]->m_immovable)
			m_bodies[i]->applyForce(m_bodies[i]->m_gravity * m_bodies[i]->m_mass);
	}


	// Collision Detection
	collide();

	// Solving
	solve(dt);

	// Integration
	for (size_t i = 0; i < m_bodies.size(); i++)
	{
		m_bodies[i]->integrate(dt);
	}
}


void lpWorld::updateBodies()
{
	for (size_t i = 0; i < m_bodies.size(); i++)
	{
		m_bodies[i]->update();
	}
}


void lpWorld::solve(float dt)
{
	//if (m_numContacts > 0) printf("%d, ", m_numContacts);
	m_solver->solve(dt, m_bodies, m_joints, m_contacts, m_numContacts, m_numDynamicBodies);
}





lpWorldSimple::lpWorldSimple(int solverType) : lpWorld(solverType)
{
}


void lpWorldSimple::collide()
{
	m_numContacts = 0;
	lpRigidBody *b1, *b2;

	//m_contacts.clear();

	// Test all pairs
	for (size_t i = 0; i < m_bodies.size() - 1; i++)
	{
		b1 = m_bodies[i];

		for (size_t j = i + 1; j < m_bodies.size(); j++)
		{
			b2 = m_bodies[j];
			if (b1->m_immovable && b2->m_immovable) continue;
			if (b1->m_freezed && b2->m_freezed) continue;

			if (b1->m_worldAABB.overlapsAABB(b2->m_worldAABB))
			{
				lpCollisions(b1->m_geometry, b2->m_geometry, &m_contacts, m_numContacts);
			}
		}

	}

}