#include "StdAfx.h"
#include "lpLCPSolver.h"


#define		LCP_BODY_NULL	-1


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
			for (i = 0; i < curDofs; i++) delete[] M[i];
			delete[] M;
			delete[] F;
			delete[] V;
			delete[] a;
		}

		M = new float*[dofs];
		for (i = 0; i < dofs; i++)
		{
			M[i] = new float[dofs];
			for (j = 0; j < dofs; j++) M[i][j] = 0.0f;
		}
		F = new float[dofs];
		V = new float[dofs];
		a = new float[dofs];

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
			delete[] lo;
			delete[] hi;
			delete[] lambda;
			delete[] diag;
		}


		J	   = new float*[s];
		Jmap   = new int*[s];
		xi	   = new float[s];
		lo	   = new float[s];
		hi	   = new float[s];
		lambda = new float[s];
		diag   = new float[s];
		At	   = new float*[s];

		for (i = 0; i < s; i++)
		{
			J[i]	= new float[jdofs];
			Jmap[i] = new int[2];
			At[i] = new float[s];
		}

		for (i = 0; i < jdofs; i++) A[i] = new float[s];

		curS = s;

		memUsed = 4 * (dofs * dofs + 3 * dofs + 5 * s + 24 * s + 2 * s + s * s);
		printf("Memory used by LCP: %f Kb\n", memUsed / 1024.0f);
	}


	float idt = 1 / dt;

	int off1, off2;

	lpRigidBody *body;



	/* 
	   Эта функция - ядро двига на основе решения Линейной Проблемы о Дополнении (LCP) для решения ограничений наложенных на тела
	   Решается следующая система относительно вектора лямбды:
	   J * M^-1 * J^T * lambda = 1/dt * xi - J(1/dt * V1 - M^-1 * F)
	*/

	

	// Строим обратную матрицу масс M^-1, в коде называется просто M
	// Строим вектор внешних сил и обобщенных скоростей
	// Вычисляем 1/dt * V - M^-1 * F
	int id = 0;
	for (i = 0, j = 0; i < numBodies; i++)
	{
		body = bodies[i];

		if (!body->m_immovable)
		{
			// Нумеруем тело
			body->m_id = id++;

			float m1 = body->m_invMass;

			M[j    ][j    ] = m1;
			M[j + 1][j + 1] = m1;
			M[j + 2][j + 2] = m1;

			body->calcInverseInertiaTensorWorld();
			lpMat3 I1 = body->m_invIWorld;
			M[j + 3][j + 3] = I1.m_data[0][0]; M[j + 3][j + 4] = I1.m_data[0][1]; M[j + 3][j + 5] = I1.m_data[0][2];
			M[j + 4][j + 3] = I1.m_data[1][0]; M[j + 4][j + 4] = I1.m_data[1][1]; M[j + 4][j + 5] = I1.m_data[1][2];
			M[j + 5][j + 3] = I1.m_data[2][0]; M[j + 5][j + 4] = I1.m_data[2][1]; M[j + 5][j + 5] = I1.m_data[2][2];	

			F[j    ] = body->m_forceAccum.m_x;
			F[j + 1] = body->m_forceAccum.m_y;
			F[j + 2] = body->m_forceAccum.m_z;
			F[j + 3] = body->m_torqueAccum.m_x;
			F[j + 4] = body->m_torqueAccum.m_y;
			F[j + 5] = body->m_torqueAccum.m_z;

			V[j    ] = body->m_linearVel.m_x;
			V[j + 1] = body->m_linearVel.m_y;
			V[j + 2] = body->m_linearVel.m_z;
			V[j + 3] = body->m_angularVel.m_x;
			V[j + 4] = body->m_angularVel.m_y;
			V[j + 5] = body->m_angularVel.m_z;

			V[j    ] = idt * V[j    ] - m1 * F[j    ];
			V[j + 1] = idt * V[j + 1] - m1 * F[j + 1];
			V[j + 2] = idt * V[j + 2] - m1 * F[j + 2];

			lpVec3 w = body->m_invIWorld * body->m_torqueAccum;
			V[j + 3] = idt * V[j + 3] - w.m_x;
			V[j + 4] = idt * V[j + 4] - w.m_y;
			V[j + 5] = idt * V[j + 5] - w.m_z;

			j += 6;
		}
	}



	// Строим Якобиан системы

	int p = 0;
	for (i = 0; i < numContacts; i++)
	{
		lpJoint *joint = contacts[i];
		float **Jpart = joint->jacobian();
		
		if (!Jpart)
		{
			// Если джойнт контакта не требует решения, то уменьшаем количество ограниченных степеней свободы
			s -= 3;
			continue;
		}

		for (int k = 0; k < 1; k++)
		{
			J[p][ 0] = Jpart[k][ 0];
			J[p][ 1] = Jpart[k][ 1];
			J[p][ 2] = Jpart[k][ 2];
			J[p][ 3] = Jpart[k][ 3];
			J[p][ 4] = Jpart[k][ 4];
			J[p][ 5] = Jpart[k][ 5];

			Jmap[p][0] = 6 * joint->m_body1->m_id;

			if (!joint->m_body2->m_immovable)
			{
				J[p][ 6] = Jpart[k][ 6];
				J[p][ 7] = Jpart[k][ 7];
				J[p][ 8] = Jpart[k][ 8];
				J[p][ 9] = Jpart[k][ 9];
				J[p][10] = Jpart[k][10];
				J[p][11] = Jpart[k][11];
				
				Jmap[p][1] = 6 * joint->m_body2->m_id;
			}
			else Jmap[p][1] = LCP_BODY_NULL;

 			xi[p] = Jpart[k][12];
			lo[p] = Jpart[k][13];
			hi[p] = Jpart[k][14];
			p++;
		}
	}


	// Если количество требуемых ограничений 0, то солвер завершает работу
	if (s == 0)
		return;


	// Вычисляем 1/dt * xi - Jq
	// A = M^-1 * J^T
	// a = A * lambda


	for (i = 0; i < dofs; i++)
	{
		a[i] = 0.0f;
	}

	for (i = 0; i < s; i++)
	{
		off1 = Jmap[i][0];
		off2 = Jmap[i][1];

		xi[i] = idt * xi[i] - (J[i][0]*V[off1] + J[i][1]*V[off1 + 1] + J[i][2]*V[off1 + 2] + J[i][3]*V[off1 + 3] + J[i][ 4]*V[off1 + 4] + J[i][ 5]*V[off1 + 5]);

		A[ 0][i] = M[off1    ][off1    ]*J[i][0] + M[off1    ][off1 + 1]*J[i][ 1] + M[off1    ][off1 + 2]*J[i][ 2] +
				   M[off1    ][off1 + 3]*J[i][3] + M[off1    ][off1 + 4]*J[i][ 4] + M[off1    ][off1 + 5]*J[i][ 5];
		A[ 1][i] = M[off1 + 1][off1    ]*J[i][0] + M[off1 + 1][off1 + 1]*J[i][ 1] + M[off1 + 1][off1 + 2]*J[i][ 2] +
				   M[off1 + 1][off1 + 3]*J[i][3] + M[off1 + 1][off1 + 4]*J[i][ 4] + M[off1 + 1][off1 + 5]*J[i][ 5];
		A[ 2][i] = M[off1 + 2][off1    ]*J[i][0] + M[off1 + 2][off1 + 1]*J[i][ 1] + M[off1 + 2][off1 + 2]*J[i][ 2] +
				   M[off1 + 2][off1 + 3]*J[i][3] + M[off1 + 2][off1 + 4]*J[i][ 4] + M[off1 + 2][off1 + 5]*J[i][ 5];
		A[ 3][i] = M[off1 + 3][off1    ]*J[i][0] + M[off1 + 3][off1 + 1]*J[i][ 1] + M[off1 + 3][off1 + 2]*J[i][ 2] +
				   M[off1 + 3][off1 + 3]*J[i][3] + M[off1 + 3][off1 + 4]*J[i][ 4] + M[off1 + 3][off1 + 5]*J[i][ 5];
		A[ 4][i] = M[off1 + 4][off1    ]*J[i][0] + M[off1 + 4][off1 + 1]*J[i][ 1] + M[off1 + 4][off1 + 2]*J[i][ 2] +
				   M[off1 + 4][off1 + 3]*J[i][3] + M[off1 + 4][off1 + 4]*J[i][ 4] + M[off1 + 4][off1 + 5]*J[i][ 5];
		A[ 5][i] = M[off1 + 5][off1    ]*J[i][0] + M[off1 + 5][off1 + 1]*J[i][ 1] + M[off1 + 5][off1 + 2]*J[i][ 2] +
				   M[off1 + 5][off1 + 3]*J[i][3] + M[off1 + 5][off1 + 4]*J[i][ 4] + M[off1 + 5][off1 + 5]*J[i][ 5];

		if (off2 != LCP_BODY_NULL)
		{
			xi[i] -= J[i][6]*V[off2] + J[i][7]*V[off2 + 1] + J[i][8]*V[off2 + 2] + J[i][9]*V[off2 + 3] + J[i][10]*V[off2 + 4] + J[i][11]*V[off2 + 5];

			A[ 0][i] += M[off1    ][off2    ]*J[i][6] + M[off1    ][off2 + 1]*J[i][ 7] + M[off1    ][off2 + 2]*J[i][ 8] +
					    M[off1    ][off2 + 3]*J[i][9] + M[off1    ][off2 + 4]*J[i][10] + M[off1    ][off2 + 5]*J[i][11];
			A[ 1][i] += M[off1 + 1][off2    ]*J[i][6] + M[off1 + 1][off2 + 1]*J[i][ 7] + M[off1 + 1][off2 + 2]*J[i][ 8] +
					    M[off1 + 1][off2 + 3]*J[i][9] + M[off1 + 1][off2 + 4]*J[i][10] + M[off1 + 1][off2 + 5]*J[i][11];
			A[ 2][i] += M[off1 + 2][off2    ]*J[i][6] + M[off1 + 2][off2 + 1]*J[i][ 7] + M[off1 + 2][off2 + 2]*J[i][ 8] +
					    M[off1 + 2][off2 + 3]*J[i][9] + M[off1 + 2][off2 + 4]*J[i][10] + M[off1 + 2][off2 + 5]*J[i][11];
			A[ 3][i] += M[off1 + 3][off2    ]*J[i][6] + M[off1 + 3][off2 + 1]*J[i][ 7] + M[off1 + 3][off2 + 2]*J[i][ 8] +
					    M[off1 + 3][off2 + 3]*J[i][9] + M[off1 + 3][off2 + 4]*J[i][10] + M[off1 + 3][off2 + 5]*J[i][11];
			A[ 4][i] += M[off1 + 4][off2    ]*J[i][6] + M[off1 + 4][off2 + 1]*J[i][ 7] + M[off1 + 4][off2 + 2]*J[i][ 8] +
					    M[off1 + 4][off2 + 3]*J[i][9] + M[off1 + 4][off2 + 4]*J[i][10] + M[off1 + 4][off2 + 5]*J[i][11];
			A[ 5][i] += M[off1 + 5][off2    ]*J[i][6] + M[off1 + 5][off2 + 1]*J[i][ 7] + M[off1 + 5][off2 + 2]*J[i][ 8] +
					    M[off1 + 5][off2 + 3]*J[i][9] + M[off1 + 5][off2 + 4]*J[i][10] + M[off1 + 5][off2 + 5]*J[i][11];

			A[ 6][i] =  M[off2    ][off1    ]*J[i][0] + M[off2    ][off1 + 1]*J[i][ 1] + M[off2    ][off1 + 2]*J[i][ 2] +
					    M[off2    ][off1 + 3]*J[i][3] + M[off2    ][off1 + 4]*J[i][ 4] + M[off2    ][off1 + 5]*J[i][ 5] +
						M[off2    ][off2    ]*J[i][6] + M[off2    ][off2 + 1]*J[i][ 7] + M[off2    ][off2 + 2]*J[i][ 8] +
						M[off2    ][off2 + 3]*J[i][9] + M[off2    ][off2 + 4]*J[i][10] + M[off2    ][off2 + 5]*J[i][11];
			A[ 7][i] =  M[off2 + 1][off1    ]*J[i][0] + M[off2 + 1][off1 + 1]*J[i][ 1] + M[off2 + 1][off1 + 2]*J[i][ 2] +
					    M[off2 + 1][off1 + 3]*J[i][3] + M[off2 + 1][off1 + 4]*J[i][ 4] + M[off2 + 1][off1 + 5]*J[i][ 5] +
						M[off2 + 1][off2    ]*J[i][6] + M[off2 + 1][off2 + 1]*J[i][ 7] + M[off2 + 1][off2 + 2]*J[i][ 8] +
						M[off2 + 1][off2 + 3]*J[i][9] + M[off2 + 1][off2 + 4]*J[i][10] + M[off2 + 1][off2 + 5]*J[i][11];
			A[ 8][i] =  M[off2 + 2][off1    ]*J[i][0] + M[off2 + 2][off1 + 1]*J[i][ 1] + M[off2 + 2][off1 + 2]*J[i][ 2] +
					    M[off2 + 2][off1 + 3]*J[i][3] + M[off2 + 2][off1 + 4]*J[i][ 4] + M[off2 + 2][off1 + 5]*J[i][ 5] +
						M[off2 + 2][off2    ]*J[i][6] + M[off2 + 2][off2 + 1]*J[i][ 7] + M[off2 + 2][off2 + 2]*J[i][ 8] +
						M[off2 + 2][off2 + 3]*J[i][9] + M[off2 + 2][off2 + 4]*J[i][10] + M[off2 + 2][off2 + 5]*J[i][11];
			A[ 9][i] =  M[off2 + 3][off1    ]*J[i][0] + M[off2 + 3][off1 + 1]*J[i][ 1] + M[off2 + 3][off1 + 2]*J[i][ 2] +
					    M[off2 + 3][off1 + 3]*J[i][3] + M[off2 + 3][off1 + 4]*J[i][ 4] + M[off2 + 3][off1 + 5]*J[i][ 5] +
						M[off2 + 3][off2    ]*J[i][6] + M[off2 + 3][off2 + 1]*J[i][ 7] + M[off2 + 3][off2 + 2]*J[i][ 8] +
						M[off2 + 3][off2 + 3]*J[i][9] + M[off2 + 3][off2 + 4]*J[i][10] + M[off2 + 3][off2 + 5]*J[i][11];
			A[10][i] =  M[off2 + 4][off1    ]*J[i][0] + M[off2 + 4][off1 + 1]*J[i][ 1] + M[off2 + 4][off1 + 2]*J[i][ 2] +
					    M[off2 + 4][off1 + 3]*J[i][3] + M[off2 + 4][off1 + 4]*J[i][ 4] + M[off2 + 4][off1 + 5]*J[i][ 5] +
						M[off2 + 4][off2    ]*J[i][6] + M[off2 + 4][off2 + 1]*J[i][ 7] + M[off2 + 4][off2 + 2]*J[i][ 8] +
						M[off2 + 4][off2 + 3]*J[i][9] + M[off2 + 4][off2 + 4]*J[i][10] + M[off2 + 4][off2 + 5]*J[i][11];
			A[11][i] =  M[off2 + 5][off1    ]*J[i][0] + M[off2 + 5][off1 + 1]*J[i][ 1] + M[off2 + 5][off1 + 2]*J[i][ 2] +
					    M[off2 + 5][off1 + 3]*J[i][3] + M[off2 + 5][off1 + 4]*J[i][ 4] + M[off2 + 5][off1 + 5]*J[i][ 5] +
						M[off2 + 5][off2    ]*J[i][6] + M[off2 + 5][off2 + 1]*J[i][ 7] + M[off2 + 5][off2 + 2]*J[i][ 8] +
						M[off2 + 5][off2 + 3]*J[i][9] + M[off2 + 5][off2 + 4]*J[i][10] + M[off2 + 5][off2 + 5]*J[i][11];
		}

	
		lambda[i] = xi[i];

		a[off1    ] += A[ 0][i] * lambda[i];
		a[off1 + 1] += A[ 1][i] * lambda[i];
		a[off1 + 2] += A[ 2][i] * lambda[i];
		a[off1 + 3] += A[ 3][i] * lambda[i];
		a[off1 + 4] += A[ 4][i] * lambda[i];
		a[off1 + 5] += A[ 5][i] * lambda[i];

		diag[i] = J[i][0] * A[0][i] + J[i][1] * A[1][i] + J[i][2] * A[2][i] + J[i][3] * A[3][i] + J[i][ 4] * A[ 4][i] + J[i][ 5] * A[ 5][i];
			

		if (off2 != LCP_BODY_NULL)
		{
			a[off2    ] += A[ 6][i] * lambda[i];
			a[off2 + 1] += A[ 7][i] * lambda[i];
			a[off2 + 2] += A[ 8][i] * lambda[i];
			a[off2 + 3] += A[ 9][i] * lambda[i];
			a[off2 + 4] += A[10][i] * lambda[i];
			a[off2 + 5] += A[11][i] * lambda[i];

			diag[i] += J[i][6] * A[6][i] + J[i][7] * A[7][i] + J[i][8] * A[8][i] + J[i][9] * A[9][i] + J[i][10] * A[10][i] + J[i][11] * A[11][i];
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


	float l, delta;

	//for (int iter = 0; iter < 10; iter++)
	//{
	//	for (i = 0; i < s; i++)
	//	{
	//		off1 = Jmap[i][0];
	//		off2 = Jmap[i][1];

	//		l = b[i];

	//		l -= J[i][0] * a[off1    ] + J[i][1] * a[off1 + 1] + J[i][2] * a[off1 + 2] + J[i][3] * a[off1 + 3] + J[i][ 4] * a[off1 + 4] + J[i][ 5] * a[off1 + 5];

	//		if (off2 != LCP_BODY_NULL)
	//		{
	//			l -= J[i][6] * a[off2    ] + J[i][7] * a[off2 + 1] + J[i][8] * a[off2 + 2] + J[i][9] * a[off2 + 3] + J[i][10] * a[off2 + 4] + J[i][11] * a[off2 + 5];
	//		}

	//		l *= diag[i];

	//		delta = lambda[i];
	//		lambda[i] += 1.2f * (l - delta);
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
				if (Jmap[i][1] != LCP_BODY_NULL)
					for (int k = 6; k < 12; k++)
					{	
						At[i][j] += J[i][k] * A[k][j];
					}
			}
		}
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
		}
		//printf("#%d:  %f\n", iter, lambda[0]);
	}

	//////////////////////////////////////////////
	

	// F + J^T * lambda
	for (i = 0; i < s; i++)
	{
		off1 = Jmap[i][0];
		off2 = Jmap[i][1];

		F[off1    ] += J[i][ 0] * lambda[i];
		F[off1 + 1] += J[i][ 1] * lambda[i];
		F[off1 + 2] += J[i][ 2] * lambda[i];
		F[off1 + 3] += J[i][ 3] * lambda[i];
		F[off1 + 4] += J[i][ 4] * lambda[i];
		F[off1 + 5] += J[i][ 5] * lambda[i];

		if (off2 != LCP_BODY_NULL)
		{
			F[off2    ] += J[i][ 6] * lambda[i];
			F[off2 + 1] += J[i][ 7] * lambda[i];
			F[off2 + 2] += J[i][ 8] * lambda[i];
			F[off2 + 3] += J[i][ 9] * lambda[i];
			F[off2 + 4] += J[i][10] * lambda[i];
			F[off2 + 5] += J[i][11] * lambda[i];
		}
	}


	for (i = 0, j = 0; i < numBodies; i++)
	{
		body = bodies[i];
		if (!body->m_immovable)
		{
			body->m_forceAccum.setTo( F[j    ], F[j + 1], F[j + 2]);
			body->m_torqueAccum.setTo(F[j + 3], F[j + 4], F[j + 5]);
			j += 6;
		}
	}


	//printf("LCP time: %f ms\n", (float)(GetTickCount() - time));
	

}