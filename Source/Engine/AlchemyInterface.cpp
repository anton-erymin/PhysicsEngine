#include "AS3.h"
AS3_Val gg_lib = NULL;

#line 1 "interface.gg"

#include "AlchemyInterface.h"

#line 6 "interface.gg"
static AS3_Val thunk_lpCreateLaxePhysicsEngine(void *gg_clientData, AS3_Val gg_args) {
	AS3_ArrayValue(gg_args, "", NULL);
	
#line 6 "interface.gg"
return AS3_Int(
#line 6 "interface.gg"
(int)lpCreateLaxePhysicsEngine());
}

#line 8 "interface.gg"
static AS3_Val thunk_lpEngineSetGravity(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	double x;
	double y;
	double z;
	AS3_ArrayValue(gg_args, "IntType, DoubleType, DoubleType, DoubleType",  &engine,  &x,  &y,  &z);
	lpEngineSetGravity(
#line 8 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 8 "interface.gg"
 x, 
#line 8 "interface.gg"
 y, 
#line 8 "interface.gg"
 z);
	return NULL;
}

#line 9 "interface.gg"
static AS3_Val thunk_lpEngineStep(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	double dt;
	AS3_ArrayValue(gg_args, "IntType, DoubleType",  &engine,  &dt);
	lpEngineStep(
#line 9 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 9 "interface.gg"
 dt);
	return NULL;
}

#line 10 "interface.gg"
static AS3_Val thunk_lpEngineAddBody(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	int body;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &engine,  &body);
	lpEngineAddBody(
#line 10 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 10 "interface.gg"
(lpRigidBody*) body);
	return NULL;
}

#line 11 "interface.gg"
static AS3_Val thunk_lpEngineRemoveBody(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	int body;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &engine,  &body);
	lpEngineRemoveBody(
#line 11 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 11 "interface.gg"
(lpRigidBody*) body);
	return NULL;
}

#line 12 "interface.gg"
static AS3_Val thunk_lpEngineCreateWorldSimple(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	int solverType;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &engine,  &solverType);
	
#line 12 "interface.gg"
return AS3_Int(
#line 12 "interface.gg"
(int)lpEngineCreateWorldSimple(
#line 12 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 12 "interface.gg"
 solverType));
}

#line 13 "interface.gg"
static AS3_Val thunk_lpEngineCreateWorldQuadtree(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	int solverType;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &engine,  &solverType);
	
#line 13 "interface.gg"
return AS3_Int(
#line 13 "interface.gg"
(int)lpEngineCreateWorldQuadtree(
#line 13 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 13 "interface.gg"
 solverType));
}

#line 14 "interface.gg"
static AS3_Val thunk_lpEngineCreateWorldBVH(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	int solverType;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &engine,  &solverType);
	
#line 14 "interface.gg"
return AS3_Int(
#line 14 "interface.gg"
(int)lpEngineCreateWorldBVH(
#line 14 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 14 "interface.gg"
 solverType));
}

#line 15 "interface.gg"
static AS3_Val thunk_lpEngineSetWorld(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	int world;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &engine,  &world);
	lpEngineSetWorld(
#line 15 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 15 "interface.gg"
(lpWorld*) world);
	return NULL;
}

#line 16 "interface.gg"
static AS3_Val thunk_lpEngineGetWorld(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	AS3_ArrayValue(gg_args, "IntType",  &engine);
	
#line 16 "interface.gg"
return AS3_Int(
#line 16 "interface.gg"
(int)lpEngineGetWorld(
#line 16 "interface.gg"
(lpLaxePhysicsEngine*) engine));
}

#line 17 "interface.gg"
static AS3_Val thunk_lpEngineCreateRigidBody(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	int addFlag;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &engine,  &addFlag);
	
#line 17 "interface.gg"
return AS3_Int(
#line 17 "interface.gg"
(int)lpEngineCreateRigidBody(
#line 17 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 17 "interface.gg"
 addFlag));
}

#line 18 "interface.gg"
static AS3_Val thunk_lpEngineCreateStaticBody(void *gg_clientData, AS3_Val gg_args) {
	int engine;
	int addFlag;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &engine,  &addFlag);
	
#line 18 "interface.gg"
return AS3_Int(
#line 18 "interface.gg"
(int)lpEngineCreateStaticBody(
#line 18 "interface.gg"
(lpLaxePhysicsEngine*) engine, 
#line 18 "interface.gg"
 addFlag));
}

#line 20 "interface.gg"
static AS3_Val thunk_lpWorldAddBody(void *gg_clientData, AS3_Val gg_args) {
	int world;
	int body;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &world,  &body);
	lpWorldAddBody(
#line 20 "interface.gg"
(lpWorld*) world, 
#line 20 "interface.gg"
(lpRigidBody*) body);
	return NULL;
}

#line 21 "interface.gg"
static AS3_Val thunk_lpWorldRemoveBody(void *gg_clientData, AS3_Val gg_args) {
	int world;
	int body;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &world,  &body);
	lpWorldRemoveBody(
#line 21 "interface.gg"
(lpWorld*) world, 
#line 21 "interface.gg"
(lpRigidBody*) body);
	return NULL;
}

#line 22 "interface.gg"
static AS3_Val thunk_lpWorldSetGravity(void *gg_clientData, AS3_Val gg_args) {
	int world;
	double x;
	double y;
	double z;
	AS3_ArrayValue(gg_args, "IntType, DoubleType, DoubleType, DoubleType",  &world,  &x,  &y,  &z);
	lpWorldSetGravity(
#line 22 "interface.gg"
(lpWorld*) world, 
#line 22 "interface.gg"
 x, 
#line 22 "interface.gg"
 y, 
#line 22 "interface.gg"
 z);
	return NULL;
}

#line 23 "interface.gg"
static AS3_Val thunk_lpWorldStep(void *gg_clientData, AS3_Val gg_args) {
	int world;
	double dt;
	AS3_ArrayValue(gg_args, "IntType, DoubleType",  &world,  &dt);
	lpWorldStep(
#line 23 "interface.gg"
(lpWorld*) world, 
#line 23 "interface.gg"
 dt);
	return NULL;
}

#line 24 "interface.gg"
static AS3_Val thunk_lpWorldUpdateBodies(void *gg_clientData, AS3_Val gg_args) {
	int world;
	AS3_ArrayValue(gg_args, "IntType",  &world);
	lpWorldUpdateBodies(
#line 24 "interface.gg"
(lpWorld*) world);
	return NULL;
}

#line 26 "interface.gg"
static AS3_Val thunk_lpRigidBodyAddGeometry(void *gg_clientData, AS3_Val gg_args) {
	int body;
	int geometry;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &body,  &geometry);
	lpRigidBodyAddGeometry(
#line 26 "interface.gg"
(lpRigidBody*) body, 
#line 26 "interface.gg"
(lpCollisionPrimitive*) geometry);
	return NULL;
}

#line 27 "interface.gg"
static AS3_Val thunk_lpRigidBodyUpdate(void *gg_clientData, AS3_Val gg_args) {
	int body;
	AS3_ArrayValue(gg_args, "IntType",  &body);
	lpRigidBodyUpdate(
#line 27 "interface.gg"
(lpRigidBody*) body);
	return NULL;
}

#line 28 "interface.gg"
static AS3_Val thunk_lpRigidBodyRotate(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double angle;
	double xaxis;
	double yaxis;
	double zaxis;
	AS3_ArrayValue(gg_args, "IntType, DoubleType, DoubleType, DoubleType, DoubleType",  &body,  &angle,  &xaxis,  &yaxis,  &zaxis);
	lpRigidBodyRotate(
#line 28 "interface.gg"
(lpRigidBody*) body, 
#line 28 "interface.gg"
 angle, 
#line 28 "interface.gg"
 xaxis, 
#line 28 "interface.gg"
 yaxis, 
#line 28 "interface.gg"
 zaxis);
	return NULL;
}

#line 29 "interface.gg"
static AS3_Val thunk_lpRigidBodySetSurfaceParams(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double restitution;
	double friction;
	AS3_ArrayValue(gg_args, "IntType, DoubleType, DoubleType",  &body,  &restitution,  &friction);
	lpRigidBodySetSurfaceParams(
#line 29 "interface.gg"
(lpRigidBody*) body, 
#line 29 "interface.gg"
 restitution, 
#line 29 "interface.gg"
 friction);
	return NULL;
}

#line 30 "interface.gg"
static AS3_Val thunk_lpRigidBodySetMass(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double mass;
	AS3_ArrayValue(gg_args, "IntType, DoubleType",  &body,  &mass);
	lpRigidBodySetMass(
#line 30 "interface.gg"
(lpRigidBody*) body, 
#line 30 "interface.gg"
 mass);
	return NULL;
}

#line 31 "interface.gg"
static AS3_Val thunk_lpRigidBodySetInertiaTensor(void *gg_clientData, AS3_Val gg_args) {
	int body;
	int tensor;
	AS3_ArrayValue(gg_args, "IntType, IntType",  &body,  &tensor);
	lpRigidBodySetInertiaTensor(
#line 31 "interface.gg"
(lpRigidBody*) body, 
#line 31 "interface.gg"
(lpMat3*) tensor);
	return NULL;
}

#line 32 "interface.gg"
static AS3_Val thunk_lpRigidBodyApplyForce(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double xforce;
	double yforce;
	double zforce;
	AS3_ArrayValue(gg_args, "IntType, DoubleType, DoubleType, DoubleType",  &body,  &xforce,  &yforce,  &zforce);
	lpRigidBodyApplyForce(
#line 32 "interface.gg"
(lpRigidBody*) body, 
#line 32 "interface.gg"
 xforce, 
#line 32 "interface.gg"
 yforce, 
#line 32 "interface.gg"
 zforce);
	return NULL;
}

#line 33 "interface.gg"
static AS3_Val thunk_lpRigidBodyApplyForceAtPoint(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double xforce;
	double yforce;
	double zforce;
	double xpoint;
	double ypoint;
	double zpoint;
	AS3_ArrayValue(gg_args, "IntType, DoubleType, DoubleType, DoubleType, DoubleType, DoubleType, DoubleType",  &body,  &xforce,  &yforce,  &zforce,  &xpoint,  &ypoint,  &zpoint);
	lpRigidBodyApplyForceAtPoint(
#line 33 "interface.gg"
(lpRigidBody*) body, 
#line 33 "interface.gg"
 xforce, 
#line 33 "interface.gg"
 yforce, 
#line 33 "interface.gg"
 zforce, 
#line 33 "interface.gg"
 xpoint, 
#line 33 "interface.gg"
 ypoint, 
#line 33 "interface.gg"
 zpoint);
	return NULL;
}

#line 34 "interface.gg"
static AS3_Val thunk_lpRigidBodyApplyTorque(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double xtorque;
	double ytorque;
	double ztorque;
	AS3_ArrayValue(gg_args, "IntType, DoubleType, DoubleType, DoubleType",  &body,  &xtorque,  &ytorque,  &ztorque);
	lpRigidBodyApplyTorque(
#line 34 "interface.gg"
(lpRigidBody*) body, 
#line 34 "interface.gg"
 xtorque, 
#line 34 "interface.gg"
 ytorque, 
#line 34 "interface.gg"
 ztorque);
	return NULL;
}

#line 35 "interface.gg"
static AS3_Val thunk_lpRigidBodyMakeImmovable(void *gg_clientData, AS3_Val gg_args) {
	int body;
	AS3_ArrayValue(gg_args, "IntType",  &body);
	lpRigidBodyMakeImmovable(
#line 35 "interface.gg"
(lpRigidBody*) body);
	return NULL;
}

#line 36 "interface.gg"
static AS3_Val thunk_lpRigidBodySetPosition(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double x;
	double y;
	double z;
	AS3_ArrayValue(gg_args, "IntType, DoubleType, DoubleType, DoubleType",  &body,  &x,  &y,  &z);
	lpRigidBodySetPosition(
#line 36 "interface.gg"
(lpRigidBody*) body, 
#line 36 "interface.gg"
 x, 
#line 36 "interface.gg"
 y, 
#line 36 "interface.gg"
 z);
	return NULL;
}

#line 37 "interface.gg"
static AS3_Val thunk_lpRigidBodyGetLinearDamping(void *gg_clientData, AS3_Val gg_args) {
	int body;
	AS3_ArrayValue(gg_args, "IntType",  &body);
	
#line 37 "interface.gg"
return AS3_Number(
#line 37 "interface.gg"
lpRigidBodyGetLinearDamping(
#line 37 "interface.gg"
(lpRigidBody*) body));
}

#line 38 "interface.gg"
static AS3_Val thunk_lpRigidBodyGetAngularDamping(void *gg_clientData, AS3_Val gg_args) {
	int body;
	AS3_ArrayValue(gg_args, "IntType",  &body);
	
#line 38 "interface.gg"
return AS3_Number(
#line 38 "interface.gg"
lpRigidBodyGetAngularDamping(
#line 38 "interface.gg"
(lpRigidBody*) body));
}

#line 39 "interface.gg"
static AS3_Val thunk_lpRigidBodySetLinearDamping(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double linDamping;
	AS3_ArrayValue(gg_args, "IntType, DoubleType",  &body,  &linDamping);
	lpRigidBodySetLinearDamping(
#line 39 "interface.gg"
(lpRigidBody*) body, 
#line 39 "interface.gg"
 linDamping);
	return NULL;
}

#line 40 "interface.gg"
static AS3_Val thunk_lpRigidBodySetAngularDamping(void *gg_clientData, AS3_Val gg_args) {
	int body;
	double angDamping;
	AS3_ArrayValue(gg_args, "IntType, DoubleType",  &body,  &angDamping);
	lpRigidBodySetAngularDamping(
#line 40 "interface.gg"
(lpRigidBody*) body, 
#line 40 "interface.gg"
 angDamping);
	return NULL;
}

#line 42 "interface.gg"
static AS3_Val thunk_lpCreateSphere(void *gg_clientData, AS3_Val gg_args) {
	double radius;
	AS3_ArrayValue(gg_args, "DoubleType",  &radius);
	
#line 42 "interface.gg"
return AS3_Int(
#line 42 "interface.gg"
(int)lpCreateSphere(
#line 42 "interface.gg"
 radius));
}

#line 43 "interface.gg"
static AS3_Val thunk_lpSphereGetVolume(void *gg_clientData, AS3_Val gg_args) {
	int sphere;
	AS3_ArrayValue(gg_args, "IntType",  &sphere);
	
#line 43 "interface.gg"
return AS3_Number(
#line 43 "interface.gg"
lpSphereGetVolume(
#line 43 "interface.gg"
(lpSphere*) sphere));
}

#line 44 "interface.gg"
static AS3_Val thunk_lpCreateBox(void *gg_clientData, AS3_Val gg_args) {
	double width;
	double height;
	double depth;
	AS3_ArrayValue(gg_args, "DoubleType, DoubleType, DoubleType",  &width,  &height,  &depth);
	
#line 44 "interface.gg"
return AS3_Int(
#line 44 "interface.gg"
(int)lpCreateBox(
#line 44 "interface.gg"
 width, 
#line 44 "interface.gg"
 height, 
#line 44 "interface.gg"
 depth));
}

#line 45 "interface.gg"
static AS3_Val thunk_lpBoxGetVolume(void *gg_clientData, AS3_Val gg_args) {
	int box;
	AS3_ArrayValue(gg_args, "IntType",  &box);
	
#line 45 "interface.gg"
return AS3_Number(
#line 45 "interface.gg"
lpBoxGetVolume(
#line 45 "interface.gg"
(lpBox*) box));
}

#line 47 "interface.gg"
static AS3_Val thunk_lpInertiaBoxTensor(void *gg_clientData, AS3_Val gg_args) {
	double mass;
	double width;
	double height;
	double depth;
	AS3_ArrayValue(gg_args, "DoubleType, DoubleType, DoubleType, DoubleType",  &mass,  &width,  &height,  &depth);
	
#line 47 "interface.gg"
return AS3_Int(
#line 47 "interface.gg"
(int)lpInertiaBoxTensor(
#line 47 "interface.gg"
 mass, 
#line 47 "interface.gg"
 width, 
#line 47 "interface.gg"
 height, 
#line 47 "interface.gg"
 depth));
}

#line 48 "interface.gg"
static AS3_Val thunk_lpInertiaSphereSolidTensor(void *gg_clientData, AS3_Val gg_args) {
	double mass;
	double radius;
	AS3_ArrayValue(gg_args, "DoubleType, DoubleType",  &mass,  &radius);
	
#line 48 "interface.gg"
return AS3_Int(
#line 48 "interface.gg"
(int)lpInertiaSphereSolidTensor(
#line 48 "interface.gg"
 mass, 
#line 48 "interface.gg"
 radius));
}

#line 49 "interface.gg"
static AS3_Val thunk_lpInertiaSphereShellTensor(void *gg_clientData, AS3_Val gg_args) {
	double mass;
	double radius;
	AS3_ArrayValue(gg_args, "DoubleType, DoubleType",  &mass,  &radius);
	
#line 49 "interface.gg"
return AS3_Int(
#line 49 "interface.gg"
(int)lpInertiaSphereShellTensor(
#line 49 "interface.gg"
 mass, 
#line 49 "interface.gg"
 radius));
}
AS3_Val gg_string(const char *str) {
	AS3_Val result = AS3_String(str);
	free((void *)str);
	return result;
}
void gg_reg(AS3_Val lib, const char *name, AS3_ThunkProc p) {
	AS3_Val fun = AS3_Function(NULL, p);
	AS3_SetS(lib, name, fun);
	AS3_Release(fun);
}
void gg_reg_async(AS3_Val lib, const char *name, AS3_ThunkProc p) {
	AS3_Val fun = AS3_FunctionAsync(NULL, p);
	AS3_SetS(lib, name, fun);
	AS3_Release(fun);
}
int main(int argc, char **argv) {
#if defined(GGINIT_DEFINED)
	ggInit();
#endif
	gg_lib = AS3_Object("");
	gg_reg(gg_lib, "lpCreateLaxePhysicsEngine", thunk_lpCreateLaxePhysicsEngine);
	gg_reg(gg_lib, "lpEngineSetGravity", thunk_lpEngineSetGravity);
	gg_reg(gg_lib, "lpEngineStep", thunk_lpEngineStep);
	gg_reg(gg_lib, "lpEngineAddBody", thunk_lpEngineAddBody);
	gg_reg(gg_lib, "lpEngineRemoveBody", thunk_lpEngineRemoveBody);
	gg_reg(gg_lib, "lpEngineCreateWorldSimple", thunk_lpEngineCreateWorldSimple);
	gg_reg(gg_lib, "lpEngineCreateWorldQuadtree", thunk_lpEngineCreateWorldQuadtree);
	gg_reg(gg_lib, "lpEngineCreateWorldBVH", thunk_lpEngineCreateWorldBVH);
	gg_reg(gg_lib, "lpEngineSetWorld", thunk_lpEngineSetWorld);
	gg_reg(gg_lib, "lpEngineGetWorld", thunk_lpEngineGetWorld);
	gg_reg(gg_lib, "lpEngineCreateRigidBody", thunk_lpEngineCreateRigidBody);
	gg_reg(gg_lib, "lpEngineCreateStaticBody", thunk_lpEngineCreateStaticBody);
	gg_reg(gg_lib, "lpWorldAddBody", thunk_lpWorldAddBody);
	gg_reg(gg_lib, "lpWorldRemoveBody", thunk_lpWorldRemoveBody);
	gg_reg(gg_lib, "lpWorldSetGravity", thunk_lpWorldSetGravity);
	gg_reg(gg_lib, "lpWorldStep", thunk_lpWorldStep);
	gg_reg(gg_lib, "lpWorldUpdateBodies", thunk_lpWorldUpdateBodies);
	gg_reg(gg_lib, "lpRigidBodyAddGeometry", thunk_lpRigidBodyAddGeometry);
	gg_reg(gg_lib, "lpRigidBodyUpdate", thunk_lpRigidBodyUpdate);
	gg_reg(gg_lib, "lpRigidBodyRotate", thunk_lpRigidBodyRotate);
	gg_reg(gg_lib, "lpRigidBodySetSurfaceParams", thunk_lpRigidBodySetSurfaceParams);
	gg_reg(gg_lib, "lpRigidBodySetMass", thunk_lpRigidBodySetMass);
	gg_reg(gg_lib, "lpRigidBodySetInertiaTensor", thunk_lpRigidBodySetInertiaTensor);
	gg_reg(gg_lib, "lpRigidBodyApplyForce", thunk_lpRigidBodyApplyForce);
	gg_reg(gg_lib, "lpRigidBodyApplyForceAtPoint", thunk_lpRigidBodyApplyForceAtPoint);
	gg_reg(gg_lib, "lpRigidBodyApplyTorque", thunk_lpRigidBodyApplyTorque);
	gg_reg(gg_lib, "lpRigidBodyMakeImmovable", thunk_lpRigidBodyMakeImmovable);
	gg_reg(gg_lib, "lpRigidBodySetPosition", thunk_lpRigidBodySetPosition);
	gg_reg(gg_lib, "lpRigidBodyGetLinearDamping", thunk_lpRigidBodyGetLinearDamping);
	gg_reg(gg_lib, "lpRigidBodyGetAngularDamping", thunk_lpRigidBodyGetAngularDamping);
	gg_reg(gg_lib, "lpRigidBodySetLinearDamping", thunk_lpRigidBodySetLinearDamping);
	gg_reg(gg_lib, "lpRigidBodySetAngularDamping", thunk_lpRigidBodySetAngularDamping);
	gg_reg(gg_lib, "lpCreateSphere", thunk_lpCreateSphere);
	gg_reg(gg_lib, "lpSphereGetVolume", thunk_lpSphereGetVolume);
	gg_reg(gg_lib, "lpCreateBox", thunk_lpCreateBox);
	gg_reg(gg_lib, "lpBoxGetVolume", thunk_lpBoxGetVolume);
	gg_reg(gg_lib, "lpInertiaBoxTensor", thunk_lpInertiaBoxTensor);
	gg_reg(gg_lib, "lpInertiaSphereSolidTensor", thunk_lpInertiaSphereSolidTensor);
	gg_reg(gg_lib, "lpInertiaSphereShellTensor", thunk_lpInertiaSphereShellTensor);
	AS3_LibInit(gg_lib);
	return 1;
}
