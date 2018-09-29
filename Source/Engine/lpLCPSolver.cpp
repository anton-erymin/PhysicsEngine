#include "StdAfx.h"
#include "lpLCPSolver.h"


#define		LCP_BODY_NULL_ID	-1
#define		LCP_BODY_NULL		-6


lpLCPSolver::lpLCPSolver()
{
	curDofs = 0;
	curS = 0;
}


void lpLCPSolver::solve(float dt, std::vector<lpRigidBody*> &bodies, std::vector<lpJoint*> &joints, 
	std::vector<lpContactJoint*> &contacts, int numContacts, int numDynamicBodies)
{
	//int time = GetTickCount();

	int numJoints = joints.size();

	if (numJoints == 0 && numContacts == 0)
		return;

	// Кол-во тел
	int n = numDynamicBodies;
	int numBodies = bodies.size();

	// Количество ограниченных степеней свободы
	int s = numJoints + 3 * numContacts;

	int i, j;

	// Кол-во степеней свободы
	int dofs = 6 * n;

	// Кол-во степеней свободы Якобиана
	int jdofs = 12;

	float memUsed;

	// Перевыделяем память если нужно
	if (dofs > curDofs)
	{
		if (curDofs > 0)
		{
			delete[] V;
			delete[] pV;
			delete[] a;
		}

		V  = new float[dofs];
		pV = new float[dofs];
		a  = new float[dofs];

		curDofs = dofs;
	}


	if (s > curS)
	{
		if (curS > 0)
		{
			for (i = 0; i < curS; i++)
			{
				delete[] J[i];
				delete[] Jmap[i];
				delete[] At[i];
			}
			delete[] J;
			delete[] Jmap;
			delete[] At;

			for (i = 0; i < jdofs; i++)
			{
				delete[] A[i];
			}
	
			delete[] xi;
			delete[] pseudoxi;
			delete[] lo;
			delete[] hi;
			delete[] lambda;
			delete[] diag;
		}


		J	     = new float*[s];
		Jmap     = new int*[s];
		xi	     = new float[s];
		pseudoxi = new float[s];
		lo	     = new float[s];
		hi	     = new float[s];
		lambda   = new float[s];
		diag     = new float[s];
		At	     = new float*[s];

		for (i = 0; i < s; i++)
		{
			J[i]	= new float[jdofs];
			Jmap[i] = new int[2];
			At[i] = new float[s];
		}

		for (i = 0; i < jdofs; i++) A[i] = new float[s];

		curS = s;

		memUsed = (float)(4 * (3 * dofs + 6 * s + 24 * s + 2 * s + s * s));
		printf("Memory used by LCP: %f Kb\n", memUsed / 1024.0f);
	}


	float idt = 1 / dt;
	float l;
	int off1, off2;

	lpRigidBody *body;



	/* 
	   Эта функция - ядро двига на основе решения Линейной Проблемы о Дополнении (LCP) для решения ограничений наложенных на тела
	   Решается следующая система относительно вектора лямбды:
	   J * M^-1 * J^T * lambda = 1/dt * xi - J(1/dt * V1 - M^-1 * F)
	*/


	// Вычисляем 1/dt * V - M^-1 * F
	int id = 0;
	for (i = 0, j = 0; i < n; i++)
	{
		body = bodies[i];

		// Нумеруем тело
		body->m_id = id++;

		float m1 = body->m_invMass;
		body->calcInverseInertiaTensorWorld();
		body->m_pseudoForce.clear();
		body->m_pseudoTorque.clear();
		

		lpVec3 f = body->m_forceAccum;
		lpVec3 t = body->m_torqueAccum;

		//body->m_pseudoForce = f;

		lpVec3 vel = body->m_linearVel;

		pV[j    ] = -m1 * f.m_x;
		pV[j + 1] = -m1 * f.m_y;
		pV[j + 2] = -m1 * f.m_z;

		V[j    ] = idt * vel.m_x + pV[j    ];
		V[j + 1] = idt * vel.m_y + pV[j + 1];
		V[j + 2] = idt * vel.m_z + pV[j + 2];

		vel = body->m_angularVel;
		lpVec3 w = body->m_invIWorld * t;

		pV[j + 3] = -w.m_x;
		pV[j + 4] = -w.m_y;
		pV[j + 5] = -w.m_z;

		V[j + 3] = idt * vel.m_x + pV[j + 3];
		V[j + 4] = idt * vel.m_y + pV[j + 4];
		V[j + 5] = idt * vel.m_z + pV[j + 5];

		j += 6;
	}



	// Строим Якобиан системы

	int p = 0;
	for (i = 0; i < numContacts; i++)
	{
		lpJoint *joint = contacts[i];
		float **Jpart = joint->jacobian(idt);
		
		if (!Jpart)
		{
			// Если джойнт контакта не требует решения, то уменьшаем количество ограниченных степеней свободы
			s -= 3;
			continue;
		}

		for (int k = 0; k < 3; k++)
		{
			J[p][ 0] = Jpart[k][ 0];
			J[p][ 1] = Jpart[k][ 1];
			J[p][ 2] = Jpart[k][ 2];
			J[p][ 3] = Jpart[k][ 3];
			J[p][ 4] = Jpart[k][ 4];
			J[p][ 5] = Jpart[k][ 5];

			Jmap[p][0] = joint->m_body1->m_id;

			if (!joint->m_body2->m_immovable)
			{
				J[p][ 6] = Jpart[k][ 6];
				J[p][ 7] = Jpart[k][ 7];
				J[p][ 8] = Jpart[k][ 8];
				J[p][ 9] = Jpart[k][ 9];
				J[p][10] = Jpart[k][10];
				J[p][11] = Jpart[k][11];
				
				Jmap[p][1] = joint->m_body2->m_id;
			}
			else Jmap[p][1] = LCP_BODY_NULL_ID;

 			xi[p]		= Jpart[k][J_XI];
			pseudoxi[p] = Jpart[k][J_PSEUDOXI];
			lo[p]		= Jpart[k][J_LO];
			hi[p]		= Jpart[k][J_HI];

			p++;
		}
	}


	// Если количество требуемых ограничений 0, то солвер завершает работу
	if (s == 0)
		return;


	// Вычисляем 1/dt * xi - J * V
	// A = M^-1 * J^T
	// a = A * lambda


	for (i = 0; i < dofs; i++)
	{
		a[i] = 0.0f;
	}

	for (i = 0; i < s; i++)
	{
		off1 = 6 * Jmap[i][0];
		off2 = 6 * Jmap[i][1];

		id = Jmap[i][0];
		float m1 = bodies[id]->m_invMass;

		A[ 0][i] = m1 * J[i][0];
		A[ 1][i] = m1 * J[i][1];
		A[ 2][i] = m1 * J[i][2];

		lpVec3 v(J[i][3], J[i][4], J[i][5]);
		v *= bodies[id]->m_invIWorld;

		A[ 3][i] = v.m_x;
		A[ 4][i] = v.m_y;
		A[ 5][i] = v.m_z;

		xi[i] = idt * xi[i] - (J[i][ 0] * V[off1    ] + J[i][ 1] * V[off1 + 1] + J[i][ 2] * V[off1 + 2] + 
							   J[i][ 3] * V[off1 + 3] + J[i][ 4] * V[off1 + 4] + J[i][ 5] * V[off1 + 5]);	

		if (off2 != LCP_BODY_NULL)
		{
			id = Jmap[i][1];
			m1 = bodies[id]->m_invMass;

			A[ 6][i] = m1 * J[i][6];
			A[ 7][i] = m1 * J[i][7];
			A[ 8][i] = m1 * J[i][8];

			lpVec3 v(J[i][9], J[i][10], J[i][11]);
			v *= bodies[id]->m_invIWorld;

			A[ 9][i] = v.m_x;
			A[10][i] = v.m_y;
			A[11][i] = v.m_z;

			xi[i] -= J[i][ 6] * V[off2    ] + J[i][ 7] * V[off2 + 1] + J[i][ 8] * V[off2 + 2] + 
					 J[i][ 9] * V[off2 + 3] + J[i][10] * V[off2 + 4] + J[i][11] * V[off2 + 5];	
		}

		lambda[i] = xi[i];
		l = lambda[i];

		a[off1++] += A[ 0][i] * l;
		a[off1++] += A[ 1][i] * l;
		a[off1++] += A[ 2][i] * l;
		a[off1++] += A[ 3][i] * l;
		a[off1++] += A[ 4][i] * l;
		a[off1++] += A[ 5][i] * l;

		diag[i] = J[i][ 0] * A[ 0][i] + J[i][ 1] * A[ 1][i] + J[i][ 2] * A[ 2][i] + 
				  J[i][ 3] * A[ 3][i] + J[i][ 4] * A[ 4][i] + J[i][ 5] * A[ 5][i];
			
		if (off2 != LCP_BODY_NULL)
		{
			a[off2++] += A[ 6][i] * l;
			a[off2++] += A[ 7][i] * l;
			a[off2++] += A[ 8][i] * l;
			a[off2++] += A[ 9][i] * l;
			a[off2++] += A[10][i] * l;
			a[off2++] += A[11][i] * l;

			diag[i] += J[i][ 6] * A[ 6][i] + J[i][ 7] * A[ 7][i] + J[i][ 8] * A[ 8][i] + 
					   J[i][ 9] * A[ 9][i] + J[i][10] * A[10][i] + J[i][11] * A[11][i];
		}
		
		diag[i] = 1.0f / diag[i];
	}


	float *b = xi;


	// Solve JAx = b
	// PGS-SOR starts here
	
	/*printf("J\n");
	for (i = 0; i < s; i++)
	{
		printf("%f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f\n", J[i][0], J[i][1], J[i][2], J[i][3], J[i][4], J[i][5], J[i][6], J[i][7], J[i][8], J[i][9],
			J[i][10], J[i][11]);
	}

	printf("\n\n\nA\n");
	for (i = 0; i < s; i++)
	{
		printf("%f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f\n", A[0][i], A[1][i], A[2][i], A[3][i], A[4][i], A[5][i], A[6][i], A[7][i], A[8][i], A[9][i],
			A[10][i], A[11][i]);
	}

	printf("\n\n\nb\n");
	for (i = 0; i < s; i++)
	{
		printf("%f  ", xi[i]);
	}
	printf("\n\n\n\n");*/


	float delta;

	//for (int iter = 0; iter < 10; iter++)
	//{
	//	for (i = 0; i < s; i++)
	//	{
	//		off1 = Jmap[i][0];
	//		off2 = Jmap[i][1];

	//		l = xi[i];

	//		l -=     J[i][ 0] * a[off1    ] + J[i][ 1] * a[off1 + 1] + J[i][ 2] * a[off1 + 2] + 
	//				 J[i][ 3] * a[off1 + 3] + J[i][ 4] * a[off1 + 4] + J[i][ 5] * a[off1 + 5];

	//		if (off2 != LCP_BODY_NULL)
	//		{
	//			l -= J[i][ 6] * a[off2    ] + J[i][ 7] * a[off2 + 1] + J[i][ 8] * a[off2 + 2] + 
	//				 J[i][ 9] * a[off2 + 3] + J[i][10] * a[off2 + 4] + J[i][11] * a[off2 + 5];
	//		}

	//		l *= diag[i];

	//		delta = lambda[i];
	//		lambda[i] += 0.8f * (l - delta);
	//		//lambda[i] = l;

	//		// Clamping
	//		if (lambda[i] < lo[i]) lambda[i] = lo[i];
	//		else if (lambda[i] > hi[i]) lambda[i] = hi[i];

	//		delta = lambda[i] - delta;

	//		printf("%f\n", lambda[i]);

	//		a[off1    ] += A[ 0][i] * delta;
	//		a[off1 + 1] += A[ 1][i] * delta;
	//		a[off1 + 2] += A[ 2][i] * delta;
	//		a[off1 + 3] += A[ 3][i] * delta;
	//		a[off1 + 4] += A[ 4][i] * delta;
	//		a[off1 + 5] += A[ 5][i] * delta;

	//		if (off2 != LCP_BODY_NULL)
	//		{
	//			a[off2    ] += A[ 6][i] * delta;
	//			a[off2 + 1] += A[ 7][i] * delta;
	//			a[off2 + 2] += A[ 8][i] * delta;
	//			a[off2 + 3] += A[ 9][i] * delta;
	//			a[off2 + 4] += A[10][i] * delta;
	//			a[off2 + 5] += A[11][i] * delta;
	//		}
	//	}
	//}



	//////////////////////////////////////////////

	for (i = 0; i < s; i++)
	{
		for (j = 0; j < s; j++)
		{
			At[i][j] = 0.0f;

			if (Jmap[i][0] == Jmap[j][0])
			{
				for (int k = 0; k < 6; k++)
				{	
					At[i][j] += J[i][k] * A[k][j];
				}
			}

			if (Jmap[i][1] == Jmap[j][1])
			{
				if (Jmap[i][1] != LCP_BODY_NULL_ID)
					for (int k = 6; k < 12; k++)
					{	
						At[i][j] += J[i][k] * A[k][j];
					}
			}
		}

		//At[i][i] += 0.0001f * idt;
	}



	for (int iter = 0; iter < 10; iter++)
	{
		for (i = 0; i < s; i++)
		{
			float sum = 0.0f;
			for (j = 0; j < i; j++)
				sum += At[i][j] * lambda[j];
			for (j = i + 1; j < s; j++)
				sum += At[i][j] * lambda[j];
			lambda[i] += ((b[i] - sum) / At[i][i] - lambda[i]) * 1.2f;

			// Clamping
			if (lambda[i] < lo[i]) lambda[i] = lo[i];
			else if (lambda[i] > hi[i]) lambda[i] = hi[i];
		}
		//printf("#%d:  %f\n", iter, lambda[0]);
	}

	//////////////////////////////////////////////
	



	// F + J^T * lambda
	for (i = 0; i < s; i++)
	{
		id = Jmap[i][0];
		l = lambda[i];

		bodies[id]->m_forceAccum.add (lpVec3(J[i][ 0] * l, J[i][ 1] * l, J[i][ 2] * l));
		bodies[id]->m_torqueAccum.add(lpVec3(J[i][ 3] * l, J[i][ 4] * l, J[i][ 5] * l));

		id = Jmap[i][1];
		if (id != LCP_BODY_NULL_ID)
		{
			bodies[id]->m_forceAccum.add (lpVec3(J[i][ 6] * l, J[i][ 7] * l, J[i][ 8] * l));
			bodies[id]->m_torqueAccum.add(lpVec3(J[i][ 9] * l, J[i][10] * l, J[i][11] * l));
		}
	}






	// PSEUDO

	//s /= 3;
	//for (i = 1, j = 3; i < s; i++)
	//{
	//	J[i][ 0] = J[j][ 0];
	//	J[i][ 1] = J[j][ 1];
	//	J[i][ 2] = J[j][ 2];
	//	J[i][ 3] = J[j][ 3];
	//	J[i][ 4] = J[j][ 4];
	//	J[i][ 5] = J[j][ 5];
	//	J[i][ 6] = J[j][ 6];
	//	J[i][ 7] = J[j][ 7];
	//	J[i][ 8] = J[j][ 8];
	//	J[i][ 9] = J[j][ 9];
	//	J[i][10] = J[j][10];
	//	J[i][11] = J[j][11];

	//	Jmap[i][0] = Jmap[j][0];
	//	Jmap[i][1] = Jmap[j][1];

	//	pseudoxi[i] = pseudoxi[j];
	//	lo[i] = lo[j];
	//	hi[i] = hi[j];

	//	j += 3;
	//}


	//for (i = 0; i < dofs; i++)
	//{
	//	a[i] = 0.0f;
	//}

	//for (i = 0; i < s; i++)
	//{
	//	off1 = 6 * Jmap[i][0];
	//	off2 = 6 * Jmap[i][1];

	//	int id = Jmap[i][0];
	//	float m1 = bodies[id]->m_invMass;

	//	A[ 0][i] = m1 * J[i][0];
	//	A[ 1][i] = m1 * J[i][1];
	//	A[ 2][i] = m1 * J[i][2];

	//	lpVec3 v(J[i][3], J[i][4], J[i][5]);
	//	v *= bodies[id]->m_invIWorld;

	//	A[ 3][i] = v.m_x;
	//	A[ 4][i] = v.m_y;
	//	A[ 5][i] = v.m_z;

	//	pseudoxi[i] = idt * pseudoxi[i] - (J[i][ 0] * pV[off1    ] + J[i][ 1] * pV[off1 + 1] + J[i][ 2] * pV[off1 + 2] + 
	//									   J[i][ 3] * pV[off1 + 3] + J[i][ 4] * pV[off1 + 4] + J[i][ 5] * pV[off1 + 5]);	

	//	if (off2 != LCP_BODY_NULL)
	//	{
	//		id = Jmap[i][1];
	//		m1 = bodies[id]->m_invMass;

	//		A[ 6][i] = m1 * J[i][6];
	//		A[ 7][i] = m1 * J[i][7];
	//		A[ 8][i] = m1 * J[i][8];

	//		lpVec3 v(J[i][9], J[i][10], J[i][11]);
	//		v *= bodies[id]->m_invIWorld;

	//		A[ 9][i] = v.m_x;
	//		A[10][i] = v.m_y;
	//		A[11][i] = v.m_z;

	//		pseudoxi[i] -= J[i][ 6] * pV[off2    ] + J[i][ 7] * pV[off2 + 1] + J[i][ 8] * pV[off2 + 2] + 
	//					   J[i][ 9] * pV[off2 + 3] + J[i][10] * pV[off2 + 4] + J[i][11] * pV[off2 + 5];	
	//	}

	//	lambda[i] = pseudoxi[i];
	//	l = lambda[i];

	//	a[off1++] += A[ 0][i] * l;
	//	a[off1++] += A[ 1][i] * l;
	//	a[off1++] += A[ 2][i] * l;
	//	a[off1++] += A[ 3][i] * l;
	//	a[off1++] += A[ 4][i] * l;
	//	a[off1++] += A[ 5][i] * l;

	//	diag[i] = J[i][ 0] * A[ 0][i] + J[i][ 1] * A[ 1][i] + J[i][ 2] * A[ 2][i] + 
	//			  J[i][ 3] * A[ 3][i] + J[i][ 4] * A[ 4][i] + J[i][ 5] * A[ 5][i];
	//		
	//	if (off2 != LCP_BODY_NULL)
	//	{
	//		a[off2++] += A[ 6][i] * l;
	//		a[off2++] += A[ 7][i] * l;
	//		a[off2++] += A[ 8][i] * l;
	//		a[off2++] += A[ 9][i] * l;
	//		a[off2++] += A[10][i] * l;
	//		a[off2++] += A[11][i] * l;

	//		diag[i] += J[i][ 6] * A[ 6][i] + J[i][ 7] * A[ 7][i] + J[i][ 8] * A[ 8][i] + 
	//				   J[i][ 9] * A[ 9][i] + J[i][10] * A[10][i] + J[i][11] * A[11][i];
	//	}
	//	
	//	diag[i] = 1.0f / diag[i];
	//}

	//b = pseudoxi;



	////////////////////////////////////////////////

	//for (i = 0; i < s; i++)
	//{
	//	for (j = 0; j < s; j++)
	//	{
	//		At[i][j] = 0.0f;

	//		if (Jmap[i][0] == Jmap[j][0])
	//		{
	//			for (int k = 0; k < 6; k++)
	//			{	
	//				At[i][j] += J[i][k] * A[k][j];
	//			}
	//		}

	//		if (Jmap[i][1] == Jmap[j][1])
	//		{
	//			if (Jmap[i][1] != LCP_BODY_NULL_ID)
	//				for (int k = 6; k < 12; k++)
	//				{	
	//					At[i][j] += J[i][k] * A[k][j];
	//				}
	//		}
	//	}
	//}

	//for (int iter = 0; iter < 100; iter++)
	//{
	//	for (i = 0; i < s; i++)
	//	{
	//		float sum = 0.0f;
	//		for (j = 0; j < i; j++)
	//			sum += At[i][j] * lambda[j];
	//		for (j = i + 1; j < s; j++)
	//			sum += At[i][j] * lambda[j];
	//		lambda[i] += ((b[i] - sum) / At[i][i] - lambda[i]) * 1.2f;

	//		// Clamping
	//		if (lambda[i] < lo[i]) lambda[i] = lo[i];
	//		else if (lambda[i] > hi[i]) lambda[i] = hi[i];
	//	}
	//	//printf("#%d:  %f\n", iter, lambda[0]);
	//}

	////////////////////////////////////////////////
	//
	//if (lambda[0] == 0.0f)
	//{
	//	int fdgdg  = 1;
	//}



	//// F + J^T * lambda
	//for (i = 0; i < s; i++)
	//{
	//	id = Jmap[i][0];
	//	l = lambda[i];

	//	bodies[id]->m_pseudoForce.add (lpVec3(J[i][ 0] * l, J[i][ 1] * l, J[i][ 2] * l));
	//	bodies[id]->m_pseudoTorque.add(lpVec3(J[i][ 3] * l, J[i][ 4] * l, J[i][ 5] * l));

	//	id = Jmap[i][1];
	//	if (id != LCP_BODY_NULL_ID)
	//	{
	//		bodies[id]->m_pseudoForce.add (lpVec3(J[i][ 6] * l, J[i][ 7] * l, J[i][ 8] * l));
	//		bodies[id]->m_pseudoTorque.add(lpVec3(J[i][ 9] * l, J[i][10] * l, J[i][11] * l));
	//	}
	//}


	//for (i = 0, j = 0; i < n; i++)
	//{
	//	body = bodies[i];

	//	body->m_pseudoForce *= body->m_invMass * dt;
	//		
	//	printf("Pseudomove: (%f; %f; %f)\n", body->m_pseudoForce.m_x, body->m_pseudoForce.m_y, body->m_pseudoForce.m_z);

	//	body->m_pos.add(body->m_pseudoForce);
	//	//body->m_pseudoForce.clear();	

	//	//m_torqueAccum *= m_invIWorld;
	//	//	
	//	//// Интегрируем уравнение на угловую скорость
	//	//// w += angAcc * dt
	//	//m_angularVel.addScaled(m_torqueAccum, dt);	
	//	//	
	//	//// Демпируем угловую скорость
	//	//m_angularVel *= pow(m_angularDamping, dt);
	//	//	
	//	//// Интегрируем уравнение на ориентацию
	//	//// R += w * dt
	//	//lpVec3 axis(m_angularVel);
	//	//float angle = axis.norm();
	//	//if (fabsf(angle) > 0.001f)
	//	//{
	//	//	axis *= 1.0f / angle;
	//	//	m_orientation.rotate(angle * dt, axis);
	//	//}
	//	//	}
	//}





	//printf("LCP time: %f ms\n", (float)(GetTickCount() - time));
	

}