#pragma once
#include "raylib.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct b2World b2World;
typedef struct b2Body b2Body;
typedef struct b2Contact b2Contact;
typedef struct b2Joint b2Joint;
typedef struct b2QueryCallback b2QueryCallback;
typedef struct b2RayCastCallback b2RayCastCallback;
// typedef struct b2DestructionListener b2DestructionListener;

typedef struct {
	bool hit;
	float nx, ny;
	float fraction;
} b2cRayCastOutput;

typedef struct {
	float step;
	float collide;
	float solve;
	float solveInit;
	float solveVelocity;
	float solvePosition;
	float broadphase;
	float solveTOI;
} b2cProfile;

typedef float(*b2cRayCastCallback)(struct b2Fixture *fixture, float x, float y, float nx, float ny, float fraction);
typedef float(*b2cQueryCallback)(struct b2Fixture *fixture);
typedef bool(*b2cContactFilter)(struct b2Fixture *fixtureA, struct b2Fixture *fixtureB);
typedef void(*b2cContactListener)(struct b2Contact *contact);
typedef void(*b2cPostContactListener)(struct b2Contact *contact);

// --------- b2World ---------------

b2World *b2World_New(float gravityX, float gravityY);
void b2World_Destroy(b2World *world);
// void b2World_SetDestructionListener(b2World *world, b2DestructionListener *listener);
void b2World_SetContactFilter(b2World *world, b2cContactFilter filter);
b2cContactFilter b2World_GetContactFilter(b2World *world);
void b2World_EnableContactFilter(b2World *world, bool flag);
void b2World_SetContactListener(b2World *world, b2cContactListener begin, b2cContactListener end, b2cContactListener preSolve, b2cPostContactListener postSolve);
b2cContactListener b2World_GetBeginContactListener(b2World *world);
b2cContactListener b2World_GetEndContactListener(b2World *world);
b2cContactListener b2World_GetPreSolveContactListener(b2World *world);
b2cPostContactListener b2World_GetPostSolveContactListener(b2World *world);

void b2World_EnableContactListener(b2World *world, bool begin, bool end, bool pre, bool post);
void b2World_SetRayCastCallback(b2World *world, b2cRayCastCallback callback);
b2cRayCastCallback b2World_GetRayCastCallback(b2World *world);
void b2World_SetQueryCallback(b2World *world, b2cQueryCallback callback);
b2cQueryCallback b2World_GetQueryCallback(b2World *world);
b2Body *b2World_CreateBody(b2World *world, float x, float y, float angle, int type);
void b2World_DestroyBody(b2World *world, b2Body *body);
void b2World_DestroyJoint(b2World *world, b2Joint *joint);
void b2World_Step(b2World *world, float timeStep, int velocityIterations, int positionIterations);
void b2World_ClearForces(b2World *world);
void b2World_QueryAABB(b2World *world, float minX, float minY, float maxX, float maxY);
b2Fixture **b2World_GetFixturesFromAABB(b2World *world, float minX, float minY, float maxX, float maxY, size_t *count);
void b2World_RayCast(b2World *world, float x1, float y1, float x2, float y2);
b2Body *b2World_GetBodyList(b2World *world);
b2Joint *b2World_GetJointList(b2World *world);
b2Contact *b2World_GetContactList(b2World *world);
void b2World_SetAllowSleeping(b2World *world, bool flag);
bool b2World_GetAllowSleeping(b2World *world);
void b2World_SetWarmStarting(b2World *world, bool flag);
bool b2World_GetWarmStarting(b2World *world);
void b2World_SetContinuousPhysics(b2World *world, bool flag);
bool b2World_GetContinuousPhysics(b2World *world);
void b2World_SetSubStepping(b2World *world, bool flag);
bool b2World_GetSubStepping(b2World *world);
int b2World_GetProxyCount(b2World *world);
int b2World_GetBodyCount(b2World *world);
int b2World_GetJointCount(b2World *world);
int b2World_GetContactCount(b2World *world);
int b2World_GetTreeHeight(b2World *world);
int b2World_GetTreeBalance(b2World *world);
float b2World_GetTreeQuality(b2World *world);
void b2World_SetGravity(b2World *world, float gravityX, float gravityY);
Vector2 *b2World_GetGravity(b2World *world, Vector2 *output);
bool b2World_IsLocked(b2World *world);
void b2World_SetAutoClearForces(b2World *world, bool flag);
bool b2World_GetAutoClearForces(b2World *world);
void b2World_ShiftOrigin(b2World *world, float x, float y);
// const b2ContactManager *b2World_GetContactManager(b2World *world);
b2cProfile *b2World_GetProfile(b2World *world, b2cProfile *output);
void b2World_Dump(b2World *world);
b2Body *b2World_GetGroundBody(b2World *world);

// --------- Shapes ---------------

typedef struct b2Shape b2Shape;

typedef struct {
	float x, y, mass, inertia;
} b2cMassData;

void b2Shape_Destroy(b2Shape *shape);
Rectangle *b2Shape_ComputeAABB(b2Shape *shape, float tx, float ty, float angle, int childIndex, Rectangle *output);
b2cMassData *b2Shape_ComputeMass(b2Shape *shape, float density, b2cMassData *output);
int b2Shape_GetType(b2Shape *shape);
int b2Shape_GetChildCount(b2Shape *shape);
bool b2Shape_TestPoint(b2Shape *shape, float tx, float ty, float angle, float x, float y);
b2cRayCastOutput *b2Shape_RayCast(b2Shape *shape, float tx, float ty, float angle, float x1, float y1,
	float x2, float y2, float maxFraction, int childIndex, b2cRayCastOutput *output);

// --------- b2CircleShape ---------------

typedef struct b2CircleShape b2CircleShape;

b2CircleShape* b2CircleShape_New(float radius);
void b2CircleShape_SetPosition(b2CircleShape *shape, float x, float y);
Vector2 *b2CircleShape_GetPosition(b2CircleShape *shape, Vector2 *output);
void b2CircleShape_SetRadius(b2CircleShape *shape, float radius);
float b2CircleShape_GetRadius(b2CircleShape *shape);

// --------- b2EdgeShape ---------------

typedef struct b2EdgeShape b2EdgeShape;

b2EdgeShape *b2EdgeShape_NewTwoSided(float x1, float y1, float x2, float y2);
b2EdgeShape *b2EdgeShape_NewOneSided(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3);
Vector2 *b2EdgeShape_GetVertex(b2EdgeShape *shape, int index, Vector2 *output);
void b2EdgeShape_SetOneSided(b2EdgeShape *shape, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3);
void b2EdgeShape_SetTwoSided(b2EdgeShape *shape, float x1, float y1, float x2, float y2);
bool b2EdgeShape_IsOneSided(b2EdgeShape *shape);

// --------- b2PolygonShape ---------------

typedef struct b2PolygonShape b2PolygonShape;

b2PolygonShape *b2PolygonShape_New(Vector2 *vertices, int count);
bool b2PolygonShape_Validate(b2PolygonShape *shape);
int b2PolygonShape_GetVertexCount(b2PolygonShape *shape);
Vector2 *b2PolygonShape_GetVertex(b2PolygonShape *shape, int index, Vector2 *output);
Vector2 *b2PolygonShape_GetNormal(b2PolygonShape *shape, int index, Vector2 *output);

// --------- b2ChainShape ---------------

typedef struct b2ChainShape b2ChainShape;

b2ChainShape *b2ChainShape_NewLoop(Vector2 *vertices, int count);
b2ChainShape *b2ChainShape_NewChain(Vector2 *vertices, int count, float x0, float y0, float x1, float y1);
int b2ChainShape_GetVertexCount(b2ChainShape *shape);
Vector2 *b2ChainShape_GetVertex(b2ChainShape *shape, int index, Vector2 *output);
Vector2 *b2ChainShape_GetPrevVertex(b2ChainShape *shape, Vector2 *output);
void b2ChainShape_SetPrevVertex(b2ChainShape *shape, float x, float y);
Vector2 *b2ChainShape_GetNextVertex(b2ChainShape *shape, Vector2 *output);
void b2ChainShape_SetNextVertex(b2ChainShape *shape, float x, float y);

// --------- Joints ---------------

int b2Joint_GetType(b2Joint *joint);
b2Body *b2Joint_GetBodyA(b2Joint *joint);
b2Body *b2Joint_GetBodyB(b2Joint *joint);
Vector2 *b2Joint_GetAnchorA(b2Joint *joint, Vector2 *output);
Vector2 *b2Joint_GetAnchorB(b2Joint *joint, Vector2 *output);
Vector2 *b2Joint_GetReactionForce(b2Joint *joint, float inv_dt, Vector2 *output);
float b2Joint_GetReactionTorque(b2Joint *joint, float inv_dt);
bool b2Joint_IsEnabled(b2Joint *joint);
bool b2Joint_GetCollideConnected(b2Joint *joint);
void b2Joint_SetUserData(b2Joint *joint, uintptr_t data);
uintptr_t b2Joint_GetUserData(b2Joint *joint);
b2Joint *b2Joint_GetNext(b2Joint *joint);
void b2Joint_ShiftOrigin(b2Joint *joint, float x, float y);
void b2Joint_Dump(b2Joint *joint);

// --------- b2RevoluteJoint ---------------

typedef struct b2RevoluteJoint b2RevoluteJoint;

b2RevoluteJoint *b2RevoluteJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, float angle, bool collide);
Vector2 *b2RevoluteJoint_GetLocalAnchorA(b2RevoluteJoint *joint, Vector2 *output);
Vector2 *b2RevoluteJoint_GetLocalAnchorB(b2RevoluteJoint *joint, Vector2 *output);
float b2RevoluteJoint_GetReferenceAngle(b2RevoluteJoint *joint);
float b2RevoluteJoint_GetJointAngle(b2RevoluteJoint *joint);
float b2RevoluteJoint_GetJointSpeed(b2RevoluteJoint *joint);
bool b2RevoluteJoint_IsLimitEnabled(b2RevoluteJoint *joint);
void b2RevoluteJoint_EnableLimit(b2RevoluteJoint *joint, bool flag);
float b2RevoluteJoint_GetLowerLimit(b2RevoluteJoint *joint);
float b2RevoluteJoint_GetUpperLimit(b2RevoluteJoint *joint);
void b2RevoluteJoint_SetLimits(b2RevoluteJoint *joint, float lower, float upper);
bool b2RevoluteJoint_IsMotorEnabled(b2RevoluteJoint *joint);
void b2RevoluteJoint_EnableMotor(b2RevoluteJoint *joint, bool flag);
void b2RevoluteJoint_SetMotorSpeed(b2RevoluteJoint *joint, float speed);
float b2RevoluteJoint_GetMotorSpeed(b2RevoluteJoint *joint);
void b2RevoluteJoint_SetMaxMotorTorque(b2RevoluteJoint *joint, float torque);
float b2RevoluteJoint_GetMaxMotorTorque(b2RevoluteJoint *joint);
float b2RevoluteJoint_GetMotorTorque(b2RevoluteJoint *joint, float inv_dt);

// --------- b2PrismaticJoint ---------------

typedef struct b2PrismaticJoint b2PrismaticJoint;

b2PrismaticJoint *b2PrismaticJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, float ax, float ay, float angle, bool collide);
Vector2 *b2PrismaticJoint_GetLocalAnchorA(b2PrismaticJoint *joint, Vector2 *output);
Vector2 *b2PrismaticJoint_GetLocalAnchorB(b2PrismaticJoint *joint, Vector2 *output);
Vector2 *b2PrismaticJoint_GetLocalAxisA(b2PrismaticJoint *joint, Vector2 *output);
float b2PrismaticJoint_GetReferenceAngle(b2PrismaticJoint *joint);
float b2PrismaticJoint_GetJointTranslation(b2PrismaticJoint *joint);
float b2PrismaticJoint_GetJointSpeed(b2PrismaticJoint *joint);
bool b2PrismaticJoint_IsLimitEnabled(b2PrismaticJoint *joint);
void b2PrismaticJoint_EnableLimit(b2PrismaticJoint *joint, bool flag);
float b2PrismaticJoint_GetLowerLimit(b2PrismaticJoint *joint);
float b2PrismaticJoint_GetUpperLimit(b2PrismaticJoint *joint);
void b2PrismaticJoint_SetLimits(b2PrismaticJoint *joint, float lower, float upper);
bool b2PrismaticJoint_IsMotorEnabled(b2PrismaticJoint *joint);
void b2PrismaticJoint_EnableMotor(b2PrismaticJoint *joint, bool flag);
void b2PrismaticJoint_SetMotorSpeed(b2PrismaticJoint *joint, float speed);
float b2PrismaticJoint_GetMotorSpeed(b2PrismaticJoint *joint);
void b2PrismaticJoint_SetMaxMotorForce(b2PrismaticJoint *joint, float force);
float b2PrismaticJoint_GetMaxMotorForce(b2PrismaticJoint *joint);
float b2PrismaticJoint_GetMotorForce(b2PrismaticJoint *joint, float inv_dt);

// --------- b2DistanceJoint ---------------

typedef struct b2DistanceJoint b2DistanceJoint;

b2DistanceJoint *b2DistanceJoint_New(b2Body *bodyA, b2Body *bodyB, float x1, float y1, float x2, float y2, bool collide);
Vector2 *b2DistanceJoint_GetLocalAnchorA(b2DistanceJoint *joint, Vector2 *output);
Vector2 *b2DistanceJoint_GetLocalAnchorB(b2DistanceJoint *joint, Vector2 *output);
float b2DistanceJoint_GetLength(b2DistanceJoint *joint);
float b2DistanceJoint_SetLength(b2DistanceJoint *joint, float length);
float b2DistanceJoint_GetMinLength(b2DistanceJoint *joint);
float b2DistanceJoint_SetMinLength(b2DistanceJoint *joint, float minLength);
float b2DistanceJoint_GetMaxLength(b2DistanceJoint *joint);
float b2DistanceJoint_SetMaxLength(b2DistanceJoint *joint, float maxLength);
float b2DistanceJoint_GetCurrentLength(b2DistanceJoint *joint);
void b2DistanceJoint_SetStiffness(b2DistanceJoint *joint, float stiffness);
float b2DistanceJoint_GetStiffness(b2DistanceJoint *joint);
void b2DistanceJoint_SetDamping(b2DistanceJoint *joint, float damping);
float b2DistanceJoint_GetDamping(b2DistanceJoint *joint);

// --------- b2PulleyJoint ---------------

typedef struct b2PulleyJoint b2PulleyJoint;

b2PulleyJoint *b2PulleyJoint_New(b2Body *bodyA, b2Body *bodyB, float gx1, float gy1, float gx2, float gy2, float x1,
	float y1, float x2, float y2, float ratio, bool collide);
Vector2 *b2PulleyJoint_GetGroundAnchorA(b2PulleyJoint *joint, Vector2 *output);
Vector2 *b2PulleyJoint_GetGroundAnchorB(b2PulleyJoint *joint, Vector2 *output);
float b2PulleyJoint_GetLengthA(b2PulleyJoint *joint);
float b2PulleyJoint_GetLengthB(b2PulleyJoint *joint);
float b2PulleyJoint_GetRatio(b2PulleyJoint *joint);
float b2PulleyJoint_GetCurrentLengthA(b2PulleyJoint *joint);
float b2PulleyJoint_GetCurrentLengthB(b2PulleyJoint *joint);

// --------- b2MouseJoint ---------------

typedef struct b2MouseJoint b2MouseJoint;

b2MouseJoint *b2MouseJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y);
void b2MouseJoint_SetTarget(b2MouseJoint *joint, float x, float y);
Vector2 *b2MouseJoint_GetTarget(b2MouseJoint *joint, Vector2 *output);
void b2MouseJoint_SetMaxForce(b2MouseJoint *joint, float force);
float b2MouseJoint_GetMaxForce(b2MouseJoint *joint);
void b2MouseJoint_SetStiffness(b2MouseJoint *joint, float stiffness);
float b2MouseJoint_GetStiffness(b2MouseJoint *joint);
void b2MouseJoint_SetDamping(b2MouseJoint *joint, float damping);
float b2MouseJoint_GetDamping(b2MouseJoint *joint);

// --------- b2GearJoint ---------------

typedef struct b2GearJoint b2GearJoint;

b2GearJoint *b2GearJoint_New(b2Joint *joint1, b2Joint *joint2, float ratio, bool collide);
b2Joint* b2GearJoint_GetJoint1(b2GearJoint *joint);
b2Joint* b2GearJoint_GetJoint2(b2GearJoint *joint);
void b2GearJoint_SetRatio(b2GearJoint *joint, float ratio);
float b2GearJoint_GetRatio(b2GearJoint *joint);

// --------- b2WheelJoint ---------------

typedef struct b2WheelJoint b2WheelJoint;

b2WheelJoint *b2WheelJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, float ax, float ay, bool collide);
Vector2 *b2WheelJoint_GetLocalAnchorA(b2WheelJoint *joint, Vector2 *output);
Vector2 *b2WheelJoint_GetLocalAnchorB(b2WheelJoint *joint, Vector2 *output);
Vector2 *b2WheelJoint_GetLocalAxisA(b2WheelJoint *joint, Vector2 *output);
float b2WheelJoint_GetJointTranslation(b2WheelJoint *joint);
float b2WheelJoint_GetJointLinearSpeed(b2WheelJoint *joint);
float b2WheelJoint_GetJointAngle(b2WheelJoint *joint);
float b2WheelJoint_GetJointAngularSpeed(b2WheelJoint *joint);
bool b2WheelJoint_IsLimitEnabled(b2WheelJoint *joint);
void b2WheelJoint_EnableLimit(b2WheelJoint *joint, bool flag);
float b2WheelJoint_GetLowerLimit(b2WheelJoint *joint);
float b2WheelJoint_GetUpperLimit(b2WheelJoint *joint);
void b2WheelJoint_SetLimits(b2WheelJoint *joint, float lower, float upper);
bool b2WheelJoint_IsMotorEnabled(b2WheelJoint *joint);
void b2WheelJoint_EnableMotor(b2WheelJoint *joint, bool flag);
void b2WheelJoint_SetMotorSpeed(b2WheelJoint *joint, float speed);
float b2WheelJoint_GetMotorSpeed(b2WheelJoint *joint);
void b2WheelJoint_SetMaxMotorTorque(b2WheelJoint *joint, float torque);
float b2WheelJoint_GetMaxMotorTorque(b2WheelJoint *joint);
float b2WheelJoint_GetMotorTorque(b2WheelJoint *joint, float inv_dt);
void b2WheelJoint_SetStiffness(b2WheelJoint *joint, float stiffness);
float b2WheelJoint_GetStiffness(b2WheelJoint *joint);
void b2WheelJoint_SetDamping(b2WheelJoint *joint, float damping);
float b2WheelJoint_GetDamping(b2WheelJoint *joint);

// --------- b2WeldJoint ---------------

typedef struct b2WeldJoint b2WeldJoint;

b2WeldJoint *b2WeldJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, float angle, bool collide);
Vector2 *b2WeldJoint_GetLocalAnchorA(b2WeldJoint *joint, Vector2 *output);
Vector2 *b2WeldJoint_GetLocalAnchorB(b2WeldJoint *joint, Vector2 *output);
float b2WeldJoint_GetReferenceAngle(b2WeldJoint *joint);
void b2WeldJoint_SetStiffness(b2WeldJoint *joint, float stiffness);
float b2WeldJoint_GetStiffness(b2WeldJoint *joint);
void b2WeldJoint_SetDamping(b2WeldJoint *joint, float damping);
float b2WeldJoint_GetDamping(b2WeldJoint *joint);

// --------- b2FrictionJoint ---------------

typedef struct b2FrictionJoint b2FrictionJoint;

b2FrictionJoint *b2FrictionJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, bool collide);
Vector2 *b2FrictionJoint_GetLocalAnchorA(b2FrictionJoint *joint, Vector2 *output);
Vector2 *b2FrictionJoint_GetLocalAnchorB(b2FrictionJoint *joint, Vector2 *output);
void b2FrictionJoint_SetMaxForce(b2FrictionJoint *joint, float force);
float b2FrictionJoint_GetMaxForce(b2FrictionJoint *joint);
void b2FrictionJoint_SetMaxTorque(b2FrictionJoint *joint, float torque);
float b2FrictionJoint_GetMaxTorque(b2FrictionJoint *joint);

// --------- b2MotorJoint ---------------

typedef struct b2MotorJoint b2MotorJoint;

b2MotorJoint *b2MotorJoint_New(b2Body *bodyA, b2Body *bodyB, bool collide);
void b2MotorJoint_SetLinearOffset(b2MotorJoint *joint, float x, float y);
Vector2 *b2MotorJoint_GetLinearOffset(b2MotorJoint *joint, Vector2 *output);
void b2MotorJoint_SetAngularOffset(b2MotorJoint *joint, float angularOffset);
float b2MotorJoint_GetAngularOffset(b2MotorJoint *joint);
void b2MotorJoint_SetMaxForce(b2MotorJoint *joint, float force);
float b2MotorJoint_GetMaxForce(b2MotorJoint *joint);
void b2MotorJoint_SetMaxTorque(b2MotorJoint *joint, float torque);
float b2MotorJoint_GetMaxTorque(b2MotorJoint *joint);
void b2MotorJoint_SetCorrectionFactor(b2MotorJoint *joint, float factor);
float b2MotorJoint_GetCorrectionFactor(b2MotorJoint *joint);

// --------- b2Fixture ---------------

typedef struct b2Fixture b2Fixture;

typedef struct {
	uint16_t category;
	uint16_t mask;
	int16_t group;
} b2cFilter;

int b2Fixture_GetType(b2Fixture *fixture);
b2Shape *b2Fixture_GetShape(b2Fixture *fixture);
void b2Fixture_SetSensor(b2Fixture *fixture, bool sensor);
bool b2Fixture_IsSensor(b2Fixture *fixture);
void b2Fixture_SetFilterData(b2Fixture *fixture, uint16_t category, uint16_t mask, int16_t group);
b2cFilter *b2Fixture_GetFilterData(b2Fixture *fixture, b2cFilter *output);
void b2Fixture_Refilter(b2Fixture *fixture);
b2Body *b2Fixture_GetBody(b2Fixture *fixture);
b2Fixture *b2Fixture_GetNext(b2Fixture *fixture);
uintptr_t b2Fixture_GetUserData(b2Fixture *fixture);
void b2Fixture_SetUserData(b2Fixture *fixture, uintptr_t data);
bool b2Fixture_TestPoint(b2Fixture *fixture, float x, float y);
b2cRayCastOutput *b2Fixture_RayCast(b2Fixture *fixture, float x1, float y1, float x2, float y2,
	float maxFraction, int childIndex, b2cRayCastOutput *output);
b2cMassData *b2Fixture_GetMassData(b2Fixture *fixture, b2cMassData *output);
void b2Fixture_SetDensity(b2Fixture *fixture, float density);
float b2Fixture_GetDensity(b2Fixture *fixture);
float b2Fixture_GetFriction(b2Fixture *fixture);
void b2Fixture_SetFriction(b2Fixture *fixture, float friction);
float b2Fixture_GetRestitution(b2Fixture *fixture);
void b2Fixture_SetRestitution(b2Fixture *fixture, float restitution);
float b2Fixture_GetRestitutionThreshold(b2Fixture *fixture);
void b2Fixture_SetRestitutionThreshold(b2Fixture *fixture, float threshold);
Rectangle *b2Fixture_GetAABB(b2Fixture *fixture, int childIndex, Rectangle *output);
void b2Fixture_Dump(b2Fixture *fixture, int bodyIndex);

// --------- b2Body ---------------

b2Fixture *b2Body_CreateFixture(b2Body *body, b2Shape *shape, float density);
void b2Body_DestroyFixture(b2Body *body, b2Fixture *fixture);
void b2Body_SetTransform(b2Body *body, float tx, float ty, float angle);
Vector2 *b2Body_GetPosition(b2Body *body, Vector2 *output);
float b2Body_GetAngle(b2Body *body);
Vector2 *b2Body_GetWorldCenter(b2Body *body, Vector2 *output);
Vector2 *b2Body_GetLocalCenter(b2Body *body, Vector2 *output);
void b2Body_SetLinearVelocity(b2Body *body, float x, float y);
Vector2 *b2Body_GetLinearVelocity(b2Body *body, Vector2 *output);
void b2Body_SetAngularVelocity(b2Body *body, float omega);
float b2Body_GetAngularVelocity(b2Body *body);
void b2Body_ApplyForce(b2Body *body, float forceX, float forceY, float x, float y, bool wake);
void b2Body_ApplyForceToCenter(b2Body *body, float forceX, float forceY, bool wake);
void b2Body_ApplyTorque(b2Body *body, float torque, bool wake);
void b2Body_ApplyLinearImpulse(b2Body *body, float impulseX, float impulseY, float x, float y, bool wake);
void b2Body_ApplyLinearImpulseToCenter(b2Body *body, float impulseX, float impulseY, bool wake);
void b2Body_ApplyAngularImpulse(b2Body *body, float impulse, bool wake);
float b2Body_GetMass(b2Body *body);
float b2Body_GetInertia(b2Body *body);
b2cMassData *b2Body_GetMassData(b2Body *body, b2cMassData *output);
void b2Body_SetMassData(b2Body *body, float x, float y, float mass, float inertia);
void b2Body_ResetMassData(b2Body *body);
Vector2 *b2Body_GetWorldPoint(b2Body *body, float localX, float localY, Vector2 *output);
Vector2 *b2Body_GetWorldVector(b2Body *body, float localX, float localY, Vector2 *output);
Vector2 *b2Body_GetLocalPoint(b2Body *body, float worldX, float worldY, Vector2 *output);
Vector2 *b2Body_GetLocalVector(b2Body *body, float worldX, float worldY, Vector2 *output);
Vector2 *b2Body_GetLinearVelocityFromWorldPoint(b2Body *body, float worldX, float worldY, Vector2 *output);
Vector2 *b2Body_GetLinearVelocityFromLocalPoint(b2Body *body, float localX, float localY, Vector2 *output);
float b2Body_GetLinearDamping(b2Body *body);
void b2Body_SetLinearDamping(b2Body *body, float linearDamping);
float b2Body_GetAngularDamping(b2Body *body);
void b2Body_SetAngularDamping(b2Body *body, float angularDamping);
float b2Body_GetGravityScale(b2Body *body);
void b2Body_SetGravityScale(b2Body *body, float scale);
void b2Body_SetType(b2Body *body, int type);
int b2Body_GetType(b2Body *body);
void b2Body_SetBullet(b2Body *body, bool flag);
bool b2Body_IsBullet(b2Body *body);
void b2Body_SetSleepingAllowed(b2Body *body, bool flag);
bool b2Body_IsSleepingAllowed(b2Body *body);
void b2Body_SetAwake(b2Body *body, bool flag);
bool b2Body_IsAwake(b2Body *body);
void b2Body_SetEnabled(b2Body *body, bool flag);
bool b2Body_IsEnabled(b2Body *body);
void b2Body_SetFixedRotation(b2Body *body, bool flag);
bool b2Body_IsFixedRotation(b2Body *body);
b2Fixture *b2Body_GetFixtureList(b2Body *body);
//b2JointEdge *b2Body_GetJointList(b2Body *body);
//b2ContactEdge *b2Body_GetContactList(b2Body *body);
b2Body *b2Body_GetNext(b2Body *body);
uintptr_t b2Body_GetUserData(b2Body *body);
void b2Body_SetUserData(b2Body *body, uintptr_t data);
b2World* b2Body_GetWorld(b2Body *body);
void b2Body_Dump(b2Body *body);

// --------- b2Contact ---------------

bool b2Contact_IsTouching(b2Contact *contact);
void b2Contact_SetEnabled(b2Contact *contact, bool flag);
bool b2Contact_IsEnabled(b2Contact *contact);
b2Contact *b2Contact_GetNext(b2Contact *contact);
b2Fixture *b2Contact_GetFixtureA(b2Contact *contact);
int b2Contact_GetChildIndexA(b2Contact *contact);
b2Fixture *b2Contact_GetFixtureB(b2Contact *contact);
int b2Contact_GetChildIndexB(b2Contact *contact);
void b2Contact_SetFriction(b2Contact *contact, float friction);
float b2Contact_GetFriction(b2Contact *contact);
void b2Contact_ResetFriction(b2Contact *contact);
void b2Contact_SetRestitution(b2Contact *contact, float restitution);
float b2Contact_GetRestitution(b2Contact *contact);
void b2Contact_ResetRestitution(b2Contact *contact);
void b2Contact_SetRestitutionThreshold(b2Contact *contact, float threshold);
float b2Contact_GetRestitutionThreshold(b2Contact *contact);
void b2Contact_ResetRestitutionThreshold(b2Contact *contact);
void b2Contact_SetTangentSpeed(b2Contact *contact, float speed);
float b2Contact_GetTangentSpeed(b2Contact *contact);
Vector2 *b2Contact_GetNormal(b2Contact *contact, Vector2* output);
int b2Contact_GetPoints(b2Contact *contact, Vector2 *output);

// --------- b2Draw ---------------

typedef struct {
	struct b2Draw *_proxy;

	void (*DrawPolygon)(const Vector2 *vertices, int vertexCount, float r, float g, float b, float a);
	void (*DrawSolidPolygon)(const Vector2 *vertices, int vertexCount, float r, float g, float b, float a);
	void (*DrawCircle)(float x, float y, float radius, float r, float g, float b, float a);
	void (*DrawSolidCircle)(float x, float y, float radius, float ax, float ay, float r, float g, float b, float a);
	void (*DrawSegment)(float x1, float y1, float x2, float y2, float r, float g, float b, float a);
	void (*DrawTransform)(float x, float y, float angle);
	void (*DrawPoint)(float x, float y, float size, float r, float g, float b, float a);
} b2cDraw;

void b2Draw_Draw(b2World *world, b2cDraw *draw);

#ifdef __cplusplus
}
#endif
