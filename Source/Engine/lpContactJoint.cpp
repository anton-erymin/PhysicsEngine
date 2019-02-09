#include "StdAfx.h"
#include "lpContactJoint.h"

#include "lpRigidBody.h"


lpContactJoint::lpContactJoint(void)
{
	m_J = new float*[3];
	m_J[0] = new float[16];
	m_J[1] = new float[16];
	m_J[2] = new float[16];

	ERP = 0.2f;
}


lpContactJoint::~lpContactJoint(void)
{
}


void lpContactJoint::prepare()
{
	// Вычисление промежуточных данных
			
	// Убеждаемся что первое тело динамическое
	if (m_body1->m_immovable)
	{
		// Если нет, меняем местами и отражаем нормаль
		lpRigidBody *temp = m_body1;
		m_body1 = m_body2;
		m_body2 = temp;
		m_normal *= -1;
	}

	// Считаем контактную матрицу
	calcContactTransform();
	
	// Относительные позиции точки контакта относительно позиций тел
	m_r1 = m_point - m_body1->m_pos;
	if (!m_body2->m_immovable) m_r2 = m_point - m_body2->m_pos;

	// Относительная скорость в точке контакта
	//calcRelativeVelocity();

	//calcDesiredDeltaVelocity();

	m_body1->calcInverseInertiaTensorWorld();
	if (!m_body2->m_immovable) m_body2->calcInverseInertiaTensorWorld();
}


void lpContactJoint::calcDesiredDeltaVelocity()
{
	m_relVelLocal = m_transform.multTransposed(m_relVel);

	// Уменьшаем коэффициент упругости, если скорость слишком мала
	float velocityLimit = 0.1f;
	float neededRestitution = m_restitution;
	if (fabsf(m_relVelLocal.m_x) < velocityLimit) neededRestitution = 0.0f;

	float lastdv = m_body1->m_deltaVel * m_normal;
	if (!m_body2->m_immovable) lastdv -= m_body2->m_deltaVel * m_normal;

	m_desiredDeltaVelocity = -m_relVelLocal.m_x - neededRestitution * (m_relVelLocal.m_x - lastdv);
}


void lpContactJoint::calcRelativeVelocity()
{
	// Вычисление относительной скорости тел в точке контакта
	m_relVel = m_body1->m_angularVel % m_r1;
	m_relVel += m_body1->m_linearVel;

	if (!m_body2->m_immovable)
	{
		m_relVel -= m_body2->m_angularVel % m_r2;
		m_relVel -= m_body2->m_linearVel;
	}
}


float lpContactJoint::calcLambda(const lpVec3 &normal, float dv)
{
	lpVec3 w = m_r1 % normal;
	float lambda = m_body1->m_invMass + (m_body1->m_invIWorld * w) * w;

	if (!m_body2->m_immovable)
	{
		w = m_r2 % normal;
		lambda += m_body2->m_invMass + (m_body2->m_invIWorld * w) * w;
	}

	return dv / lambda;
}


void lpContactJoint::solveVelocity()
{
	if (m_relVelLocal.m_x > 0) return;

	// Находим величины импульсов	
	lpVec3 impulse;
	impulse.m_x = calcLambda(m_normal, m_desiredDeltaVelocity);
	impulse.m_y = calcLambda(m_frictionDir1, -m_relVelLocal.m_y);
	impulse.m_z = calcLambda(m_frictionDir2, -m_relVelLocal.m_z);

	// Проверим нужно ли переходить к динамическому трению и перейти в режим скольжения
	float planarImpulse = sqrtf(impulse.m_y * impulse.m_y + impulse.m_z * impulse.m_z);
	float limit = m_friction * impulse.m_x;
	if (planarImpulse > limit)
	{
		impulse.m_y /= planarImpulse;
		impulse.m_z /= planarImpulse;

		impulse.m_y *= limit;
		impulse.m_z *= limit;
	}

	// Применяем импульсы
	impulse *= m_transform;

	m_velChange[0] = impulse;
	m_velChange[0] *= m_body1->m_invMass;
	m_angVelChange[0] = m_r1 % impulse;
	m_angVelChange[0] *= m_body1->m_invIWorld;
	m_body1->m_linearVel += m_velChange[0];
	m_body1->m_angularVel += m_angVelChange[0];

	if (!m_body2->m_immovable)
	{
		impulse *= -1;

		m_velChange[1] = impulse;
		m_velChange[1] *= m_body2->m_invMass;
		m_angVelChange[1] = m_r2 % impulse;
		m_angVelChange[1] *= m_body2->m_invIWorld;
		m_body2->m_linearVel += m_velChange[1];
		m_body2->m_angularVel += m_angVelChange[1];
	}
}


void lpContactJoint::solvePenetration()
{
	if (m_depth <= 0) return;

	float angInertia1, angInertia2, angMove1, angMove2;
	float linInertia1, linInertia2, linMove1, linMove2;
	float total;

	// Some magic here...

	lpVec3 inertia = m_r1 % m_normal;
	inertia *= m_body1->m_invIWorld;
	inertia %= m_r1;
	angInertia1 = inertia * m_normal;
	linInertia1 = m_body1->m_invMass;

	total = linInertia1 + angInertia1;

	if (!m_body2->m_immovable)
	{
		inertia = m_r2 % m_normal;
		inertia *= m_body2->m_invIWorld;
		inertia %= m_r2;
		angInertia2 = inertia * m_normal;
		linInertia2 = m_body2->m_invMass;

		total += linInertia2 + angInertia2;
	}

	total = 1.0f / total;
	linMove1 = 0.5f * m_depth * linInertia1 * total;
	angMove1 = 0.5f * m_depth * angInertia1 * total;
	if (!m_body2->m_immovable)
	{
		linMove2 = -0.5f * m_depth * linInertia2 * total;
		angMove2 = -0.5f * m_depth * angInertia2 * total;
	}
	

	lpVec3 move;
	float limit, totalMove;
	float angLimit = 0.2f;

	if (angMove1 != 0.0f)
	{
		limit = angLimit * m_r1.norm();
		if (fabsf(angMove1) > limit)
		{
			totalMove = linMove1 + angMove1;
			if (angMove1 >= 0.0f) 
				angMove1 = limit;
			else angMove1 = -limit;

			linMove1 = totalMove - angMove1;
		}

		move = m_r1 % m_normal;
		move *= m_body1->m_invIWorld;
		move *= angMove1 / angInertia1;

		m_orChange[0] = move;

		float angle = move.norm();
		if (angle != 0.0f)
		{
			move *= 1 / angle;
			m_body1->m_orientation.rotate(angle, move);
		}
	}
	
	move = m_normal * linMove1;
	m_posChange[0] = move;
	m_body1->m_pos += move;


	if (!m_body2->m_immovable)
	{
		if (angMove2 != 0.0f)
		{
			limit = angLimit * m_r2.norm();
			if (fabsf(angMove2) > limit)
			{
				totalMove = linMove2 + angMove2;
				if (angMove2 >= 0.0f) 
					angMove2 = limit;
				else angMove2 = -limit;

				linMove2 = totalMove - angMove2;
			}

			move = m_r2 % m_normal;
			move *= m_body2->m_invIWorld;
			move *= angMove2 / angInertia2;

			m_orChange[1] = move;

			float angle = move.norm();
			if (angle != 0.0f)
			{
				move *= 1 / angle;
				m_body2->m_orientation.rotate(angle, move);
			}
		}
	
		move = m_normal * linMove2;
		m_posChange[1] = move;
		m_body2->m_pos += move;
	}

}


void lpContactJoint::calcContactTransform()
{
	float s;
	if (fabsf(m_normal.m_x) > fabsf(m_normal.m_y))
	{
		s = 1.0f / sqrtf(m_normal.m_z * m_normal.m_z + m_normal.m_x * m_normal.m_x);

		m_frictionDir2.m_x = s * m_normal.m_z;
		m_frictionDir2.m_y = 0.0f;
		m_frictionDir2.m_z = -s * m_normal.m_x;

		m_frictionDir1.m_x = m_normal.m_y * m_frictionDir2.m_x;
		m_frictionDir1.m_y = m_normal.m_z * m_frictionDir2.m_x - m_normal.m_x * m_frictionDir2.m_z;
		m_frictionDir1.m_z = -m_normal.m_y * m_frictionDir2.m_x;
	}
	else
	{
		s = 1.0f / sqrtf(m_normal.m_z * m_normal.m_z + m_normal.m_y * m_normal.m_y);

		m_frictionDir2.m_x = 0.0f;
		m_frictionDir2.m_y = -s * m_normal.m_z;
		m_frictionDir2.m_z = s * m_normal.m_y;

		m_frictionDir1.m_x = m_normal.m_y * m_frictionDir2.m_z - m_normal.m_z * m_frictionDir2.m_y;
		m_frictionDir1.m_y = -m_normal.m_x * m_frictionDir2.m_z;
		m_frictionDir1.m_z = m_normal.m_x * m_frictionDir2.m_y;
	}

	m_transform.m_data[0][0] = m_normal.m_x;
	m_transform.m_data[1][0] = m_normal.m_y;
	m_transform.m_data[2][0] = m_normal.m_z;

	m_transform.m_data[0][1] = m_frictionDir1.m_x;
	m_transform.m_data[1][1] = m_frictionDir1.m_y;
	m_transform.m_data[2][1] = m_frictionDir1.m_z;

	m_transform.m_data[0][2] = m_frictionDir2.m_x;
	m_transform.m_data[1][2] = m_frictionDir2.m_y;
	m_transform.m_data[2][2] = m_frictionDir2.m_z;
}


float** lpContactJoint::jacobian(float idt)
{
	// Убеждаемся что первое тело динамическое
	if (m_body1->m_immovable)
	{
		// Если нет, меняем местами и отражаем нормаль
		lpRigidBody *temp = m_body1;
		m_body1 = m_body2;
		m_body2 = temp;
		m_normal *= -1.0f;
	}

	// Относительные позиции точки контакта относительно позиций тел
	m_r1 = m_point - m_body1->m_pos;
	if (!m_body2->m_immovable) m_r2 = m_point - m_body2->m_pos;

	// Вычисление относительной скорости тел в точке контакта	
	m_relVel = m_body1->m_angularVel % m_r1;
	m_relVel += m_body1->m_linearVel;

	if (!m_body2->m_immovable)
	{
		m_relVel -= m_body2->m_angularVel % m_r2;
		m_relVel -= m_body2->m_linearVel;
	}

	float velProj = m_relVel * m_normal;
	if (velProj > 0.0f)
		return 0;


	// Джойнт расталкивания тел
	lpVec3 w = m_r1 % m_normal;
	m_J[0][0] = m_normal.m_x;  m_J[0][1] = m_normal.m_y;   m_J[0][2] = m_normal.m_z; 
	m_J[0][3] = w.m_x;		   m_J[0][4] = w.m_y;		   m_J[0][5] = w.m_z; 
	w = m_r2 % m_normal;
	m_J[0][6] = -m_normal.m_x; m_J[0][ 7] = -m_normal.m_y; m_J[0][ 8] = -m_normal.m_z; 
	m_J[0][9] = -w.m_x;		   m_J[0][10] = -w.m_y;		   m_J[0][11] = -w.m_z; 
	// xi
	
	if (fabsf(velProj) < 0.1f) m_restitution = 0.0f;

	m_J[0][J_XI]	   = -velProj * m_restitution;
	if (m_depth > 0.2f)
		m_J[0][J_XI] += ERP * m_depth * idt;
	m_J[0][J_PSEUDOXI] = m_depth;
	m_J[0][J_LO]	   = 0.0f;
	m_J[0][J_HI]	   = 1e+38f;


	// Находим тангенциальные вектора трения
	calcContactTransform();

	// Джойнт трения 1
	w = m_r1 % m_frictionDir1;
	m_J[1][0] = m_frictionDir1.m_x;  m_J[1][1] = m_frictionDir1.m_y;   m_J[1][2] = m_frictionDir1.m_z; 
	m_J[1][3] = w.m_x;				 m_J[1][4] = w.m_y;				   m_J[1][5] = w.m_z; 
	w = m_r2 % m_frictionDir1;
	m_J[1][6] = -m_frictionDir1.m_x; m_J[1][ 7] = -m_frictionDir1.m_y; m_J[1][ 8] = -m_frictionDir1.m_z; 
	m_J[1][9] = -w.m_x;				 m_J[1][10] = -w.m_y;			   m_J[1][11] = -w.m_z; 

	m_J[1][J_XI] = 0.0f;
	m_J[1][J_LO] = -1e+38f;
	m_J[1][J_HI] = 1e+38f;




	// Джойнт трения 2
	w = m_r1 % m_frictionDir2;
	m_J[2][0] = m_frictionDir2.m_x;  m_J[2][1] = m_frictionDir2.m_y;   m_J[2][2] = m_frictionDir2.m_z; 
	m_J[2][3] = w.m_x;				 m_J[2][4] = w.m_y;				   m_J[2][5] = w.m_z; 
	w = m_r2 % m_frictionDir2;
	m_J[2][6] = -m_frictionDir2.m_x; m_J[2][ 7] = -m_frictionDir2.m_y; m_J[2][ 8] = -m_frictionDir2.m_z; 
	m_J[2][9] = -w.m_x;				 m_J[2][10] = -w.m_y;			   m_J[2][11] = -w.m_z; 

	m_J[2][J_XI] = 0.0f;
	m_J[2][J_LO] = -1e+38f;
	m_J[2][J_HI] = 1e+38f;

	return m_J;
}