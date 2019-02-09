#include "StdAfx.h"
#include "lpSequentialSolver.h"



lpSequentialSolver::lpSequentialSolver(void)
{
}

lpSequentialSolver::~lpSequentialSolver(void)
{
}



void lpSequentialSolver::solve(float dt, std::vector<lpRigidBody*> &bodies, std::vector<lpJoint*> &joints, std::vector<lpContactJoint*> &contacts, int numContacts, int numDynamicBodies)
{
	// –азрешение контактов

	if (numContacts == 0) return;

	prepareContacts(contacts, numContacts);
	solveVelocities(contacts, numContacts);
	solvePenetrations(contacts, numContacts);
}


void lpSequentialSolver::prepareContacts(std::vector<lpContactJoint*> &contacts, int numContacts)
{
	for (int i = 0; i < numContacts; i++)
	{
		contacts[i]->prepare();
	}
}


void lpSequentialSolver::solvePenetrations(std::vector<lpContactJoint*> &contacts, int numContacts)
{
	int itUsed = 0;
	lpContactJoint *neededContact;
	lpVec3 cp;

	int maxIter = 10;

	for (itUsed = 0; itUsed < maxIter; itUsed++)
	{
		for (int i = 0; i < numContacts; i++)
		{
			neededContact = contacts[i];
			if (neededContact->m_depth > 0.01f)
			{
				neededContact->solvePenetration();

				// ƒалее обновл€ем проникновени€ дл€ всех контактов с которыми св€зан текущий
				for (int i = 0; i < numContacts; i++)
				{
					lpContactJoint *c = contacts[i];

					if (!c->m_body1->m_immovable)
					{
						if (c->m_body1 == neededContact->m_body1)
						{
							cp = neededContact->m_orChange[0] % c->m_r1;
							cp += neededContact->m_posChange[0];
							c->m_depth -= cp * c->m_normal;
						}
						else if (c->m_body1 == neededContact->m_body2)
						{
							cp = neededContact->m_orChange[1] % c->m_r1;
							cp += neededContact->m_posChange[1];
							c->m_depth -= cp * c->m_normal;
						}
					}

					if (!c->m_body2->m_immovable)
					{
						if (c->m_body2 == neededContact->m_body1)
						{
							cp = neededContact->m_orChange[0] % c->m_r2;
							cp += neededContact->m_posChange[0];
							c->m_depth += cp * c->m_normal;
						}
						else if (c->m_body2 == neededContact->m_body2)
						{
							cp = neededContact->m_orChange[1] % c->m_r2;
							cp += neededContact->m_posChange[1];
							c->m_depth += cp * c->m_normal;
						}
					}
				}
			}
		}
	}



	//while (itUsed < maxIter)
	//{
	//	neededContact = 0;
	//	max = 0.01f;

	//	// »щем контакт с наибольшей глубиной проникновени€
	//	for (int i = 0; i < numContacts; i++)
	//	{
	//		if (contacts[i]->m_depth > max)
	//		{
	//			neededContact = contacts[i];
	//			max = contacts[i]->m_depth;
	//		}
	//	}

	//	if (!neededContact) break;

	//	// ≈сли такой найден, решаем проникновение
	//	neededContact->solvePenetration();

	//	// ƒалее обновл€ем проникновени€ дл€ всех контактов с которыми св€зан текущий
	//	for (int i = 0; i < numContacts; i++)
	//	{
	//		lpContactJoint *c = contacts[i];

	//		if (!c->m_body1->m_immovable)
	//		{
	//			if (c->m_body1 == neededContact->m_body1)
	//			{
	//				cp = neededContact->m_orChange[0] % c->m_r1;
	//				cp += neededContact->m_posChange[0];
	//				c->m_depth -= cp * c->m_normal;
	//			}
	//			else if (c->m_body1 == neededContact->m_body2)
	//			{
	//				cp = neededContact->m_orChange[1] % c->m_r1;
	//				cp += neededContact->m_posChange[1];
	//				c->m_depth -= cp * c->m_normal;
	//			}
	//		}

	//		if (!c->m_body2->m_immovable)
	//		{
	//			if (c->m_body2 == neededContact->m_body1)
	//			{
	//				cp = neededContact->m_orChange[0] % c->m_r2;
	//				cp += neededContact->m_posChange[0];
	//				c->m_depth += cp * c->m_normal;
	//			}
	//			else if (c->m_body2 == neededContact->m_body2)
	//			{
	//				cp = neededContact->m_orChange[1] % c->m_r2;
	//				cp += neededContact->m_posChange[1];
	//				c->m_depth += cp * c->m_normal;
	//			}
	//		}
	//	}

	//	itUsed++;
	//}//while

}


void lpSequentialSolver::solveVelocities(std::vector<lpContactJoint*> &contacts, int numContacts)
{
	int itUsed = 0;
	lpContactJoint *neededContact;
	lpVec3 cp;

	int maxIter = 20;

	for (itUsed = 0; itUsed < maxIter; itUsed++)
	{
		for (int i = 0; i < numContacts; i++)
		{
			neededContact = contacts[i];
			neededContact->calcRelativeVelocity();
			neededContact->calcDesiredDeltaVelocity();
			neededContact->solveVelocity();
		}
	}



	//while (itUsed < maxIter)
	//{
	//	neededContact = 0;
	//	max = 0.01f;

	//	// »щем контакт с наибольшей скоростью
	//	for (int i = 0; i < numContacts; i++)
	//	{
	//		if (contacts[i]->m_desiredDeltaVelocity > max)
	//		{
	//			neededContact = contacts[i];
	//			max = contacts[i]->m_desiredDeltaVelocity;
	//		}
	//	}

	//	if (!neededContact) break;

	//	// ≈сли такой найден, решаем скорости
	//	neededContact->solveVelocity();

	//	// ƒалее обновл€ем скорости дл€ всех контактов с которыми св€зан текущий
	//	for (int i = 0; i < numContacts; i++)
	//	{
	//		lpContactJoint *c = contacts[i];

	//		if (!c->m_body1->m_immovable)
	//		{
	//			if (c->m_body1 == neededContact->m_body1)
	//			{
	//				cp = neededContact->m_angVelChange[0] % c->m_r1;
	//				cp += neededContact->m_velChange[0];
	//				c->m_relVel += cp;
	//			}
	//			else if (c->m_body1 == neededContact->m_body2)
	//			{
	//				cp = neededContact->m_angVelChange[1] % c->m_r1;
	//				cp += neededContact->m_velChange[1];
	//				c->m_relVel += cp;
	//			}
	//		}

	//		if (!c->m_body2->m_immovable)
	//		{
	//			if (c->m_body2 == neededContact->m_body1)
	//			{
	//				cp = neededContact->m_angVelChange[0] % c->m_r2;
	//				cp += neededContact->m_velChange[0];
	//				c->m_relVel -= cp;
	//			}
	//			else if (c->m_body2 == neededContact->m_body2)
	//			{
	//				cp = neededContact->m_angVelChange[1] % c->m_r2;
	//				cp += neededContact->m_velChange[1];
	//				c->m_relVel -= cp;
	//			}
	//		}

	//		c->calcDesiredDeltaVelocity();

	//	}

	//	itUsed++;

	//}//while

}