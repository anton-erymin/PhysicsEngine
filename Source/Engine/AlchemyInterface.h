#define COMPILE_ALCHEMY

#include "lpMath.cpp"
#include "lpLaxePhysicsEngine.cpp"
#include "lpWorld.cpp"
#include "lpBroadPhase.cpp"
#include "lpNarrowPhase.cpp"
#include "lpCollisionTest.cpp"
#include "lpJoint.cpp"
#include "lpContactJoint.cpp"
#include "lpInertia.cpp"
#include "lpLCPSolver.cpp"
#include "lpRigidBody.cpp"
#include "lpSequentialSolver.cpp"
#include "lpSolver.cpp"
#include "flash.cpp"


lpLaxePhysicsEngine* lpCreateLaxePhysicsEngine()
{
	return new lpLaxePhysicsEngine();
}


void lpEngineSetGravity(lpLaxePhysicsEngine *engine, float x, float y, float z)
{
	engine->setGravity(lpVec3(x, y, z));
}


void lpEngineStep(lpLaxePhysicsEngine *engine, float dt)
{
	engine->step(dt);
}


void lpEngineAddBody(lpLaxePhysicsEngine *engine, lpRigidBody *body)
{
	engine->addBody(body);
}


void lpEngineRemoveBody(lpLaxePhysicsEngine *engine, lpRigidBody *body)
{
	engine->removeBody(body);
}


lpWorld* lpEngineCreateWorldSimple(lpLaxePhysicsEngine *engine, int solverType)
{
	return engine->createWorldSimple(solverType);
}


lpWorld* lpEngineCreateWorldQuadtree(lpLaxePhysicsEngine *engine, int solverType)
{
	return engine->createWorldQuadtree(solverType);
}


lpWorld* lpEngineCreateWorldBVH(lpLaxePhysicsEngine *engine, int solverType)
{
	return engine->createWorldBVH(solverType);
}


void lpEngineSetWorld(lpLaxePhysicsEngine *engine, lpWorld *world)
{
	engine->setWorld(world);
}


lpWorld* lpEngineGetWorld(lpLaxePhysicsEngine *engine)
{
	return engine->getWorld();
}


lpRigidBody* lpEngineCreateRigidBody(lpLaxePhysicsEngine *engine, bool addFlag)
{
	return engine->createRigidBody(addFlag);
}


lpRigidBody* lpEngineCreateStaticBody(lpLaxePhysicsEngine *engine, bool addFlag)
{
	return engine->createStaticBody(addFlag);
}







void lpWorldAddBody(lpWorld *world, lpRigidBody *body)
{
	world->addBody(body);
}


void lpWorldRemoveBody(lpWorld *world, lpRigidBody *body)
{
	world->removeBody(body);
}


void lpWorldSetGravity(lpWorld *world, float x, float y, float z)
{
	world->setGravity(lpVec3(x, y, z));
}


void lpWorldStep(lpWorld *world, float dt)
{
	world->step(dt);
}


void lpWorldUpdateBodies(lpWorld *world)
{
	world->updateBodies();
}





void lpRigidBodyAddGeometry(lpRigidBody* body, lpCollisionPrimitive* geometry)
{
	body->addGeometry(geometry);
}


void lpRigidBodyUpdate(lpRigidBody* body)
{
	body->update();
}


void lpRigidBodyRotate(lpRigidBody* body, float angle, float xaxis, float yaxis, float zaxis)
{
	body->rotate(angle, lpVec3(xaxis, yaxis, zaxis));
}


void lpRigidBodySetSurfaceParams(lpRigidBody* body, float restitution, float friction)
{
	body->setSurfaceParams(restitution, friction);
}


void lpRigidBodySetMass(lpRigidBody* body, float mass)
{
	body->setMass(mass);
}


void lpRigidBodySetInertiaTensor(lpRigidBody* body, lpMat3* tensor)
{
	body->setInertiaTensor(*tensor);
}


void lpRigidBodyApplyForce(lpRigidBody* body, float xforce, float yforce, float zforce)
{
	body->applyForce(lpVec3(xforce, yforce, zforce));
}


void lpRigidBodyApplyForceAtPoint(lpRigidBody* body, float xforce, float yforce, float zforce, float xpoint, float ypoint, float zpoint)
{
	body->applyForceAtPoint(lpVec3(xforce, yforce, zforce), lpVec3(xpoint, ypoint, zpoint));
}


void lpRigidBodyApplyTorque(lpRigidBody* body, float xtorque, float ytorque, float ztorque)
{
	body->applyTorque(lpVec3(xtorque, ytorque, ztorque));
}


void lpRigidBodyMakeImmovable(lpRigidBody* body)
{
	body->makeImmovable();
}


void lpRigidBodySetPosition(lpRigidBody* body, float x, float y, float z)
{
	body->setPosition(lpVec3(x, y, z));
}


float lpRigidBodyGetLinearDamping(lpRigidBody *body)
{
	return body->getLinearDamping();
}


float lpRigidBodyGetAngularDamping(lpRigidBody *body)
{
	return body->getAngularDamping();
}


void lpRigidBodySetLinearDamping(lpRigidBody *body, float linDamping)
{
	body->setLinearDamping(linDamping);
}


void lpRigidBodySetAngularDamping(lpRigidBody *body, float angDamping)
{
	body->setAngularDamping(angDamping);
}




lpSphere* lpCreateSphere(float radius)
{
	return new lpSphere(radius);
}


float lpSphereGetVolume(lpSphere* sphere)
{
	return sphere->getVolume();
}


lpBox* lpCreateBox(float width, float height, float depth)
{
	return new lpBox(width, height, depth);
}


float lpBoxGetVolume(lpBox* box)
{
	return box->getVolume();
}


lpMat3* lpInertiaBoxTensor(float mass, float width, float height, float depth)
{
	lpMat3 *t = new lpMat3();
	*t = lpInertia::boxTensor(mass, width, height, depth);
	return t;
}


lpMat3* lpInertiaSphereSolidTensor(float mass, float radius)
{
	lpMat3 *t = new lpMat3();
	*t = lpInertia::sphereSolidTensor(mass, radius);
	return t;
}


lpMat3* lpInertiaSphereShellTensor(float mass, float radius)
{
	lpMat3 *t = new lpMat3();
	*t = lpInertia::sphereShellTensor(mass, radius);
	return t;
}