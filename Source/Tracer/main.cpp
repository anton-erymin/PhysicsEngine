// test.cpp : Defines the entry point for the console application.
//

#include <gl/glut.h>
#include "LaxePhysicsEngine.h"
#include <Windows.h>


// Половина ширины пола
#define			FLOOR_SIZE		100.0f

// Радиус шара
#define			BALL_RADIUS		2.0f

// Кол-во шаров
#define			NUM_BALLS		16



lpLaxePhysicsEngine *lpe;
lpRigidBody **balls;

lpRigidBody *box;


float dt = 1.0f / 60.0f;
DWORD lastTime, newTime;
float accTime, frameTime;
	

bool keymap[256];


float cameraX = 200.0f;



void draw();
void reshape(int, int);
void idle();
void keyboard(unsigned char, int, int);
void keyboardUp(unsigned char, int, int);


void initGL()
{
	// Цвет очистки буфера
	glClearColor(0.8f, 0.8f, 0.8f, 0.0f);
	// Значение для очитски буфера глубины
	glClearDepth(1.0f);
	// Режим закраски полигонов
	glShadeModel(GL_SMOOTH);

	// Включаем антиалиасинг
	glEnable(GL_LINE_SMOOTH);                     
	// Настраиваем его
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	// Включаем смешение цветов и настраиваем функцию смешения
	glEnable(GL_BLEND);                                 
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Включаем тест глубины
	glEnable(GL_DEPTH_TEST);
	// Функция теста
	glDepthFunc(GL_LEQUAL);

	// Включаем освещение и нулевой источник света
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}



void buildWall(const lpVec3 &boxSize, float x, float z)
{
	lpRigidBody *wall = lpe->createStaticBody(true);
	wall->setSurfaceParams(0.2f, 1.0f);
	lpBox *gwall = new lpBox(boxSize);
	wall->addGeometry(gwall);
	wall->m_localAABB.m_radius = gwall->m_radius;
	wall->m_pos.m_x = x;
	wall->m_pos.m_z = z;
	wall->update();
}


void pyramide()
{
	if (NUM_BALLS >= 16)
	{
		balls[0]->m_pos.setTo(-50, BALL_RADIUS, 0);

		float r = BALL_RADIUS + 0.05f;

		int i = 0;
		float x = 50.0f;	
		int n = 5;
		float z = n * 2.0f * r / 2.0f - r;
		float dx = sqrtf(3.0f) * r;
			
		while (i < 15)
		{
			for (int j = n; j >= 1; j--)
			{	
				balls[i + 1]->m_pos.setTo(x, BALL_RADIUS, z - 2.0f * r * (n - j ));
				i++;
			}

			x -= dx;
			z -= r;
			n -= 1;
		}
	}
}


void initPhysics()
{
	lpe = new lpLaxePhysicsEngine();
	lpe->setWorld(lpe->createWorldSimple(SOLVER_SEQUENTIAL));
	lpe->setGravity(lpVec3(0.0f, -28.0f, 0.0f));


	balls = new lpRigidBody*[NUM_BALLS];
	float y = 30.0f;
	for (int i = 0; i < NUM_BALLS; i++)
	{
		balls[i] = lpe->createRigidBody(true);
		balls[i]->setMass(10.0f);
		balls[i]->setInertiaTensor(lpInertia::sphereSolidTensor(2.0f, BALL_RADIUS));
		balls[i]->setPosition(1.0f, y, 0.0f);
		balls[i]->setSurfaceParams(0.7f, 0.5f);
		balls[i]->setAngularDamping(0.5f);
		balls[i]->setLinearDamping(0.5f);
		lpSphere *sph = new lpSphere(BALL_RADIUS);
		balls[i]->addGeometry(sph);
		balls[i]->m_localAABB.m_radius.setTo(BALL_RADIUS, BALL_RADIUS, BALL_RADIUS);
		y += 2*BALL_RADIUS + 0.1f;

		//balls[i]->m_pos.m_x = 0.2f * ((float)rand() / (float)RAND_MAX) - 0.1f;
		
		/*balls[i]->m_pos.m_x = 2 * FLOOR_SIZE * ((float)rand() / (float)RAND_MAX) - FLOOR_SIZE;
		balls[i]->m_pos.m_z = 2 * FLOOR_SIZE * ((float)rand() / (float)RAND_MAX) - FLOOR_SIZE;
		balls[i]->m_pos.m_y = 200.0f * ((float)rand() / (float)RAND_MAX) + 10.0f;*/

		//balls[i]->m_pos.clear();
		//balls[i]->m_pos.m_y = 100;
		//balls[i]->m_linearVel.m_y = -100.0f;
	}

	//balls[0]->m_pos.m_y = 10.0f;
	//balls[1]->m_pos.m_y = 40.0f;
	//balls[0]->setMass(100.0f);

	pyramide();

	lpRigidBody *floor = lpe->createStaticBody(true);
	floor->setSurfaceParams(0.0f, 0.9f);
	floor->m_pos.m_y = -25.0f;
	lpBox *gfloor = new lpBox(2.0f * FLOOR_SIZE, 50.0f, 2.0f * FLOOR_SIZE);
	floor->addGeometry(gfloor);
	floor->m_localAABB.m_radius.setTo(FLOOR_SIZE, 25.0f, FLOOR_SIZE);
	floor->update();


	buildWall(lpVec3(2.0f * FLOOR_SIZE, 500000.0f, 50.0f), 0.0f, FLOOR_SIZE + 25.0f);
	buildWall(lpVec3(2.0f * FLOOR_SIZE, 500000.0f, 50.0f), 0.0f, -FLOOR_SIZE - 25.0f);
	buildWall(lpVec3(50.0f, 500000.0f, 2.0f * FLOOR_SIZE), FLOOR_SIZE + 25.0f, 0.0f);
	buildWall(lpVec3(50.0f, 500000.0f, 2.0f * FLOOR_SIZE), -FLOOR_SIZE - 25.0f, 0.0f);



	int width = int(BALL_RADIUS);
	int height = int(BALL_RADIUS);
	int depth = int(BALL_RADIUS);


	/*box = lpe->createRigidBody(true);
	box->setMass(10.0f);
	box->setInertiaTensor(lpInertia::boxTensor(3.0f, width, height, depth));
	box->setPosition(0.0f, 160.0f, 0.0f);
	box->setSurfaceParams(0.9f, 0.7f);
	lpBox *gbox = new lpBox(width, height, depth);
	box->addGeometry(gbox);
	box->m_localAABB.m_radius = gbox->m_radius;

	lpVec3 axis = LP_X_AXIS + LP_Y_AXIS;
	axis.normalize();
	box->rotate(0.5f, axis);*/

	


	accTime = 0.0f;
	lastTime = GetTickCount();
}


int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(300, 150);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	glutCreateWindow("Laxe Physics Engine Demo 1.0");

	glutDisplayFunc(draw);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutKeyboardUpFunc(keyboardUp);

	initGL();
	initPhysics();

	glutMainLoop();
}


void drawFloor()
{
	//floor
	
	glBegin(GL_TRIANGLE_STRIP);
		glNormal3f(0.0f, 1.0f, 0.0f);
		glColor3i(0, 255, 0);
		glVertex3f(FLOOR_SIZE, 0.0f, FLOOR_SIZE);
		glVertex3f(FLOOR_SIZE, 0.0f, -FLOOR_SIZE);
		glVertex3f(-FLOOR_SIZE, 0.0f, FLOOR_SIZE);
		glVertex3f(-FLOOR_SIZE, 0.0f, -FLOOR_SIZE);
	glEnd();
}


void drawBall(lpRigidBody *ball)
{
	glPushMatrix();

	float m[16];
	m[0 ] = ball->m_orientation.m_data[0][0];
	m[1 ] = ball->m_orientation.m_data[1][0];
	m[2 ] = ball->m_orientation.m_data[2][0];
	m[3 ] = 0.0f;

	m[4 ] = ball->m_orientation.m_data[0][1];
	m[5 ] = ball->m_orientation.m_data[1][1];
	m[6 ] = ball->m_orientation.m_data[2][1];
	m[7 ] = 0.0f;

	m[8 ] = ball->m_orientation.m_data[0][2];
	m[9 ] = ball->m_orientation.m_data[1][2];
	m[10] = ball->m_orientation.m_data[2][2];
	m[11] = 0.0f;

	m[12] = ball->m_pos.m_x;
	m[13] = ball->m_pos.m_y;
	m[14] = ball->m_pos.m_z;
	m[15] = 1.0f;

	glMultMatrixf(m);
	//glTranslatef(ball->m_pos.m_x, ball->m_pos.m_y, ball->m_pos.m_z);

	//glutWireSphere(BALL_RADIUS, 8, 8);
	glutSolidSphere(BALL_RADIUS, 12, 6);
	glPopMatrix();
}


void drawBox(lpRigidBody *box)
{
	glPushMatrix();

	float m[16];
	m[0 ] = box->m_orientation.m_data[0][0];
	m[1 ] = box->m_orientation.m_data[1][0];
	m[2 ] = box->m_orientation.m_data[2][0];
	m[3 ] = 0.0f;

	m[4 ] = box->m_orientation.m_data[0][1];
	m[5 ] = box->m_orientation.m_data[1][1];
	m[6 ] = box->m_orientation.m_data[2][1];
	m[7 ] = 0.0f;

	m[8 ] = box->m_orientation.m_data[0][2];
	m[9 ] = box->m_orientation.m_data[1][2];
	m[10] = box->m_orientation.m_data[2][2];
	m[11] = 0.0f;

	m[12] = box->m_pos.m_x;
	m[13] = box->m_pos.m_y;
	m[14] = box->m_pos.m_z;
	m[15] = 1.0f;

	glMultMatrixf(m);
	//glTranslatef(box->m_pos.m_x, box->m_pos.m_y, box->m_pos.m_z);

	glutSolidCube(BALL_RADIUS);
	glPopMatrix();
}



void draw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	gluLookAt(cameraX, 100.0f, -120.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);

	drawFloor();
	for (int i = 0; i < NUM_BALLS; i++) drawBall(balls[i]);
	//drawBox(box);
	
	glutSwapBuffers();  
}


void reshape(int width, int height)
{
	// Вьюпорт на весь экран
	glViewport(0, 0, 800, 600);

	// Редактируем матрицу проекции
	glMatrixMode(GL_PROJECTION);
	// Ставимм единичную
	glLoadIdentity();
	// Перспективная проекция
	gluPerspective(45.0f, (float)800 / (float) 600, 1.0f, 10000.0f);
	// Назад в режим матрицы вида
	glMatrixMode(GL_MODELVIEW);
}


void idle()
{
	if (keymap['w'])
	{
		if (balls[0]->m_linearVel.m_x < 0.0f)
			balls[0]->m_linearVel.m_x = 0.0f;
		else balls[0]->m_linearVel.m_x += 0.5f;
	}

	if (keymap['s'])
	{
		if (balls[0]->m_linearVel.m_x > 0.0f)
			balls[0]->m_linearVel.m_x = 0.0f;
		else balls[0]->m_linearVel.m_x -= 0.5f;
	}

	if (keymap['a'])
	{
		if (balls[0]->m_angularVel.m_y < 0.0f)
			balls[0]->m_angularVel.m_y = 0.0f;
		else balls[0]->m_angularVel.m_y += 0.3f;
	}

	if (keymap['d'])
	{
		if (balls[0]->m_angularVel.m_y > 0.0f)
			balls[0]->m_angularVel.m_y = 0.0f;
		else balls[0]->m_angularVel.m_y -= 0.3f;
	}


	if (keymap['+'])
	{
		cameraX -= 5.0f;
	}

	if (keymap['-'])
	{
		cameraX += 5.0f;
	}






	newTime = GetTickCount();
	frameTime = (newTime - lastTime) * 0.001f;
	lastTime = newTime;

	float ft = frameTime;
	if (frameTime > 0.1f) frameTime = 0.1f;
	accTime += frameTime;
	while (accTime > dt)
	{
		lpe->step(dt);
		accTime -= dt;
		//printf("%s   %f\n", balls[0]->m_freezed ? "freezed" : "NO", balls[0]->m_energy);
		//printf("velY: %f\n", balls[0]->m_linearVel.m_y);
	}

	

	glutPostRedisplay();


	static int lastUpdate = 0;
	static int frames = 0;
	char buf[256];

	int currentTime = GetTickCount();
	frames++;

	if (currentTime - lastUpdate >= 1000)
	{
		sprintf(buf, "Laxe Physics Engine Demo 1.0. [FPS: %d, FTime: %f s.]", frames, ft);
		glutSetWindowTitle(buf);
		frames = 0;
		lastUpdate = currentTime;
	}

	
	
}


void keyboard(unsigned char key, int x, int y)
{
	keymap[key] = true;
}


void keyboardUp(unsigned char key, int x, int y)
{
	keymap[key] = false;
}