package laxe.physics {
	public class LaxePhysicsEngineDLL {
		import laxe.physics.CLibInit;
		protected static const _lib_init:laxe.physics.CLibInit = new laxe.physics.CLibInit();
		protected static const _lib:* = _lib_init.init();
		static public function lpCreateLaxePhysicsEngine():int {
			return _lib.lpCreateLaxePhysicsEngine();
		}
		static public function lpEngineSetGravity(engine:int, x:Number, y:Number, z:Number):void {
			_lib.lpEngineSetGravity(engine, x, y, z);
		}
		static public function lpEngineStep(engine:int, dt:Number):void {
			_lib.lpEngineStep(engine, dt);
		}
		static public function lpEngineAddBody(engine:int, body:int):void {
			_lib.lpEngineAddBody(engine, body);
		}
		static public function lpEngineRemoveBody(engine:int, body:int):void {
			_lib.lpEngineRemoveBody(engine, body);
		}
		static public function lpEngineCreateWorldSimple(engine:int, solverType:int):int {
			return _lib.lpEngineCreateWorldSimple(engine, solverType);
		}
		static public function lpEngineCreateWorldQuadtree(engine:int, solverType:int):int {
			return _lib.lpEngineCreateWorldQuadtree(engine, solverType);
		}
		static public function lpEngineCreateWorldBVH(engine:int, solverType:int):int {
			return _lib.lpEngineCreateWorldBVH(engine, solverType);
		}
		static public function lpEngineSetWorld(engine:int, world:int):void {
			_lib.lpEngineSetWorld(engine, world);
		}
		static public function lpEngineGetWorld(engine:int):int {
			return _lib.lpEngineGetWorld(engine);
		}
		static public function lpEngineCreateRigidBody(engine:int, addFlag:Boolean):int {
			return _lib.lpEngineCreateRigidBody(engine, addFlag);
		}
		static public function lpEngineCreateStaticBody(engine:int, addFlag:Boolean):int {
			return _lib.lpEngineCreateStaticBody(engine, addFlag);
		}
		static public function lpWorldAddBody(world:int, body:int):void {
			_lib.lpWorldAddBody(world, body);
		}
		static public function lpWorldRemoveBody(world:int, body:int):void {
			_lib.lpWorldRemoveBody(world, body);
		}
		static public function lpWorldSetGravity(world:int, x:Number, y:Number, z:Number):void {
			_lib.lpWorldSetGravity(world, x, y, z);
		}
		static public function lpWorldStep(world:int, dt:Number):void {
			_lib.lpWorldStep(world, dt);
		}
		static public function lpWorldUpdateBodies(world:int):void {
			_lib.lpWorldUpdateBodies(world);
		}
		static public function lpRigidBodyAddGeometry(body:int, geometry:int):void {
			_lib.lpRigidBodyAddGeometry(body, geometry);
		}
		static public function lpRigidBodyUpdate(body:int):void {
			_lib.lpRigidBodyUpdate(body);
		}
		static public function lpRigidBodyRotate(body:int, angle:Number, xaxis:Number, yaxis:Number, zaxis:Number):void {
			_lib.lpRigidBodyRotate(body, angle, xaxis, yaxis, zaxis);
		}
		static public function lpRigidBodySetSurfaceParams(body:int, restitution:Number, friction:Number):void {
			_lib.lpRigidBodySetSurfaceParams(body, restitution, friction);
		}
		static public function lpRigidBodySetMass(body:int, mass:Number):void {
			_lib.lpRigidBodySetMass(body, mass);
		}
		static public function lpRigidBodySetInertiaTensor(body:int, tensor:int):void {
			_lib.lpRigidBodySetInertiaTensor(body, tensor);
		}
		static public function lpRigidBodyApplyForce(body:int, xforce:Number, yforce:Number, zforce:Number):void {
			_lib.lpRigidBodyApplyForce(body, xforce, yforce, zforce);
		}
		static public function lpRigidBodyApplyForceAtPoint(body:int, xforce:Number, yforce:Number, zforce:Number, xpoint:Number, ypoint:Number, zpoint:Number):void {
			_lib.lpRigidBodyApplyForceAtPoint(body, xforce, yforce, zforce, xpoint, ypoint, zpoint);
		}
		static public function lpRigidBodyApplyTorque(body:int, xtorque:Number, ytorque:Number, ztorque:Number):void {
			_lib.lpRigidBodyApplyTorque(body, xtorque, ytorque, ztorque);
		}
		static public function lpRigidBodyMakeImmovable(body:int):void {
			_lib.lpRigidBodyMakeImmovable(body);
		}
		static public function lpRigidBodySetPosition(body:int, x:Number, y:Number, z:Number):void {
			_lib.lpRigidBodySetPosition(body, x, y, z);
		}
		static public function lpRigidBodyGetLinearDamping(body:int):Number {
			return _lib.lpRigidBodyGetLinearDamping(body);
		}
		static public function lpRigidBodyGetAngularDamping(body:int):Number {
			return _lib.lpRigidBodyGetAngularDamping(body);
		}
		static public function lpRigidBodySetLinearDamping(body:int, linDamping:Number):void {
			_lib.lpRigidBodySetLinearDamping(body, linDamping);
		}
		static public function lpRigidBodySetAngularDamping(body:int, angDamping:Number):void {
			_lib.lpRigidBodySetAngularDamping(body, angDamping);
		}
		static public function lpCreateSphere(radius:Number):int {
			return _lib.lpCreateSphere(radius);
		}
		static public function lpSphereGetVolume(sphere:int):Number {
			return _lib.lpSphereGetVolume(sphere);
		}
		static public function lpCreateBox(width:Number, height:Number, depth:Number):int {
			return _lib.lpCreateBox(width, height, depth);
		}
		static public function lpBoxGetVolume(box:int):Number {
			return _lib.lpBoxGetVolume(box);
		}
		static public function lpInertiaBoxTensor(mass:Number, width:Number, height:Number, depth:Number):int {
			return _lib.lpInertiaBoxTensor(mass, width, height, depth);
		}
		static public function lpInertiaSphereSolidTensor(mass:Number, radius:Number):int {
			return _lib.lpInertiaSphereSolidTensor(mass, radius);
		}
		static public function lpInertiaSphereShellTensor(mass:Number, radius:Number):int {
			return _lib.lpInertiaSphereShellTensor(mass, radius);
		}
	}
}
