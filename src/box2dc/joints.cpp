#include "b2_world.h"
#include "b2_body.h"
#include "b2_joint.h"
#include "b2_revolute_joint.h"
#include "b2_prismatic_joint.h"
#include "b2_distance_joint.h"
#include "b2_pulley_joint.h"
#include "b2_mouse_joint.h"
#include "b2_gear_joint.h"
#include "b2_wheel_joint.h"
#include "b2_weld_joint.h"
#include "b2_friction_joint.h"
#include "b2_motor_joint.h"
#include "box2dc.h"
#include "util.h"

int b2Joint_GetType(b2Joint *joint) { return joint->GetType(); }

b2Body *b2Joint_GetBodyA(b2Joint *joint) { return joint->GetBodyA(); }
b2Body *b2Joint_GetBodyB(b2Joint *joint) { return joint->GetBodyB(); }

Vector2 *b2Joint_GetAnchorA(b2Joint *joint, Vector2 *output) {
	return convertVector(joint->GetAnchorA(), output);
}

Vector2 *b2Joint_GetAnchorB(b2Joint *joint, Vector2 *output) {
	return convertVector(joint->GetAnchorB(), output);
}

Vector2 *b2Joint_GetReactionForce(b2Joint *joint, float inv_dt, Vector2 *output) {
	return convertVector(joint->GetReactionForce(inv_dt), output);
}

float b2Joint_GetReactionTorque(b2Joint *joint, float inv_dt) { return joint->GetReactionTorque(inv_dt); }
bool b2Joint_IsEnabled(b2Joint *joint) { return joint->IsEnabled(); }
bool b2Joint_GetCollideConnected(b2Joint *joint) { return joint->GetCollideConnected(); }
void b2Joint_SetUserData(b2Joint *joint, uintptr_t data) { joint->GetUserData().pointer = data; }
uintptr_t b2Joint_GetUserData(b2Joint *joint) { return joint->GetUserData().pointer; }
b2Joint *b2Joint_GetNext(b2Joint *joint) { return joint->GetNext(); }
void b2Joint_ShiftOrigin(b2Joint *joint, float x, float y) { joint->ShiftOrigin(b2Vec2(x, y)); }
void b2Joint_Dump(b2Joint *joint) { joint->Dump(); }

// --------- b2RevoluteJoint ---------------

b2RevoluteJoint *b2RevoluteJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, float angle, bool collide) {
	b2RevoluteJointDef def;
	def.Initialize(bodyA, bodyB, b2Vec2(x, y));
	def.referenceAngle = angle;
	def.collideConnected = collide;
	return static_cast<b2RevoluteJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

Vector2 *b2RevoluteJoint_GetLocalAnchorA(b2RevoluteJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorA(), output);
}

Vector2 *b2RevoluteJoint_GetLocalAnchorB(b2RevoluteJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorB(), output);
}

float b2RevoluteJoint_GetReferenceAngle(b2RevoluteJoint *joint) { return joint->GetReferenceAngle(); }
float b2RevoluteJoint_GetJointAngle(b2RevoluteJoint *joint) { return joint->GetJointAngle(); }
float b2RevoluteJoint_GetJointSpeed(b2RevoluteJoint *joint) { return joint->GetJointSpeed(); }
bool b2RevoluteJoint_IsLimitEnabled(b2RevoluteJoint *joint) { return joint->IsLimitEnabled(); }
void b2RevoluteJoint_EnableLimit(b2RevoluteJoint *joint, bool flag) { joint->EnableLimit(flag); }
float b2RevoluteJoint_GetLowerLimit(b2RevoluteJoint *joint) { return joint->GetLowerLimit(); }
float b2RevoluteJoint_GetUpperLimit(b2RevoluteJoint *joint) { return joint->GetUpperLimit(); }
void b2RevoluteJoint_SetLimits(b2RevoluteJoint *joint, float lower, float upper) { joint->SetLimits(lower, upper); }
bool b2RevoluteJoint_IsMotorEnabled(b2RevoluteJoint *joint) { return joint->IsMotorEnabled(); }
void b2RevoluteJoint_EnableMotor(b2RevoluteJoint *joint, bool flag) { joint->EnableMotor(flag); }
void b2RevoluteJoint_SetMotorSpeed(b2RevoluteJoint *joint, float speed) { joint->SetMotorSpeed(speed); }
float b2RevoluteJoint_GetMotorSpeed(b2RevoluteJoint *joint) { return joint->GetMotorSpeed(); }
void b2RevoluteJoint_SetMaxMotorTorque(b2RevoluteJoint *joint, float torque) { joint->SetMaxMotorTorque(torque); }
float b2RevoluteJoint_GetMaxMotorTorque(b2RevoluteJoint *joint) { return joint->GetMaxMotorTorque(); }
float b2RevoluteJoint_GetMotorTorque(b2RevoluteJoint *joint, float inv_dt) { return joint->GetMotorTorque(inv_dt); }

// --------- b2PrismaticJoint ---------------

b2PrismaticJoint *b2PrismaticJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, float ax, float ay, float angle, bool collide) {
	b2PrismaticJointDef def;
	def.Initialize(bodyA, bodyB, b2Vec2(x, y), b2Vec2(ax, ay));
	def.collideConnected = collide;
	def.referenceAngle = angle;
	return static_cast<b2PrismaticJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

Vector2 *b2PrismaticJoint_GetLocalAnchorA(b2PrismaticJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorA(), output);
}

Vector2 *b2PrismaticJoint_GetLocalAnchorB(b2PrismaticJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorB(), output);
}

Vector2 *b2PrismaticJoint_GetLocalAxisA(b2PrismaticJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAxisA(), output);
}

float b2PrismaticJoint_GetReferenceAngle(b2PrismaticJoint *joint) { return joint->GetReferenceAngle(); }
float b2PrismaticJoint_GetJointTranslation(b2PrismaticJoint *joint) { return joint->GetJointTranslation(); }
float b2PrismaticJoint_GetJointSpeed(b2PrismaticJoint *joint) { return joint->GetJointSpeed(); }
bool b2PrismaticJoint_IsLimitEnabled(b2PrismaticJoint *joint) { return joint->IsLimitEnabled(); }
void b2PrismaticJoint_EnableLimit(b2PrismaticJoint *joint, bool flag) { joint->EnableLimit(flag); }
float b2PrismaticJoint_GetLowerLimit(b2PrismaticJoint *joint) { return joint->GetLowerLimit(); }
float b2PrismaticJoint_GetUpperLimit(b2PrismaticJoint *joint) { return joint->GetUpperLimit(); }
void b2PrismaticJoint_SetLimits(b2PrismaticJoint *joint, float lower, float upper) { joint->SetLimits(lower, upper); }
bool b2PrismaticJoint_IsMotorEnabled(b2PrismaticJoint *joint) { return joint->IsMotorEnabled(); }
void b2PrismaticJoint_EnableMotor(b2PrismaticJoint *joint, bool flag) { joint->EnableMotor(flag); }
void b2PrismaticJoint_SetMotorSpeed(b2PrismaticJoint *joint, float speed) { joint->SetMotorSpeed(speed); }
float b2PrismaticJoint_GetMotorSpeed(b2PrismaticJoint *joint) { return joint->GetMotorSpeed(); }
void b2PrismaticJoint_SetMaxMotorForce(b2PrismaticJoint *joint, float force) { joint->SetMaxMotorForce(force); }
float b2PrismaticJoint_GetMaxMotorForce(b2PrismaticJoint *joint) { return joint->GetMaxMotorForce(); }
float b2PrismaticJoint_GetMotorForce(b2PrismaticJoint *joint, float inv_dt) { return joint->GetMotorForce(inv_dt); }

// --------- b2DistanceJoint ---------------

b2DistanceJoint *b2DistanceJoint_New(b2Body *bodyA, b2Body *bodyB, float x1, float y1, float x2, float y2, bool collide) {
	b2DistanceJointDef def;
	def.Initialize(bodyA, bodyB, b2Vec2(x1, y1), b2Vec2(x2, y2));
	def.collideConnected = collide;
	return static_cast<b2DistanceJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

Vector2 *b2DistanceJoint_GetLocalAnchorA(b2DistanceJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorA(), output);
}

Vector2 *b2DistanceJoint_GetLocalAnchorB(b2DistanceJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorB(), output);
}

float b2DistanceJoint_GetLength(b2DistanceJoint *joint) { return joint->GetLength(); }
float b2DistanceJoint_SetLength(b2DistanceJoint *joint, float length) { return joint->SetLength(length); }
float b2DistanceJoint_GetMinLength(b2DistanceJoint *joint) { return joint->GetMinLength(); }
float b2DistanceJoint_SetMinLength(b2DistanceJoint *joint, float minLength) { return joint->SetMinLength(minLength); }
float b2DistanceJoint_GetMaxLength(b2DistanceJoint *joint) { return joint->GetMaxLength(); }
float b2DistanceJoint_SetMaxLength(b2DistanceJoint *joint, float maxLength) { return joint->SetMaxLength(maxLength); }
float b2DistanceJoint_GetCurrentLength(b2DistanceJoint *joint) { return joint->GetCurrentLength(); }
void b2DistanceJoint_SetStiffness(b2DistanceJoint *joint, float stiffness) { joint->SetStiffness(stiffness); }
float b2DistanceJoint_GetStiffness(b2DistanceJoint *joint) { return joint->GetStiffness(); }
void b2DistanceJoint_SetDamping(b2DistanceJoint *joint, float damping) { joint->SetDamping(damping); }
float b2DistanceJoint_GetDamping(b2DistanceJoint *joint) { return joint->GetDamping(); }

// --------- b2PulleyJoint ---------------

b2PulleyJoint *b2PulleyJoint_New(b2Body *bodyA, b2Body *bodyB, float gx1, float gy1, float gx2, float gy2, float x1,
	float y1, float x2, float y2, float ratio, bool collide) {
	b2PulleyJointDef def;
	def.Initialize(bodyA, bodyB, b2Vec2(gx1, gy1), b2Vec2(gx2, gy2), b2Vec2(x1, y1), b2Vec2(x2, y2), ratio);
	def.collideConnected = collide;
	return static_cast<b2PulleyJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

Vector2 *b2PulleyJoint_GetGroundAnchorA(b2PulleyJoint *joint, Vector2 *output) {
	return convertVector(joint->GetGroundAnchorA(), output);
}

Vector2 *b2PulleyJoint_GetGroundAnchorB(b2PulleyJoint *joint, Vector2 *output) {
	return convertVector(joint->GetGroundAnchorB(), output);
}

float b2PulleyJoint_GetLengthA(b2PulleyJoint *joint) { return joint->GetLengthA(); }
float b2PulleyJoint_GetLengthB(b2PulleyJoint *joint) { return joint->GetLengthB(); }
float b2PulleyJoint_GetRatio(b2PulleyJoint *joint) { return joint->GetRatio(); }
float b2PulleyJoint_GetCurrentLengthA(b2PulleyJoint *joint) { return joint->GetCurrentLengthA(); }
float b2PulleyJoint_GetCurrentLengthB(b2PulleyJoint *joint) { return joint->GetCurrentLengthB(); }

// --------- b2MouseJoint ---------------

b2MouseJoint *b2MouseJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y) {
	b2MouseJointDef def;
	def.target.x = x;
	def.target.y = y;
	def.bodyA = bodyA;
	def.bodyB = bodyB;
	return static_cast<b2MouseJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

void b2MouseJoint_SetTarget(b2MouseJoint *joint, float x, float y) { joint->SetTarget(b2Vec2(x, y)); }
Vector2 *b2MouseJoint_GetTarget(b2MouseJoint *joint, Vector2 *output) {
	return convertVector(joint->GetTarget(), output);
}

void b2MouseJoint_SetMaxForce(b2MouseJoint *joint, float force) { joint->SetMaxForce(force); }
float b2MouseJoint_GetMaxForce(b2MouseJoint *joint) { return joint->GetMaxForce(); }
void b2MouseJoint_SetStiffness(b2MouseJoint *joint, float stiffness) { joint->SetStiffness(stiffness); }
float b2MouseJoint_GetStiffness(b2MouseJoint *joint) { return joint->GetStiffness(); }
void b2MouseJoint_SetDamping(b2MouseJoint *joint, float damping) { joint->SetDamping(damping); }
float b2MouseJoint_GetDamping(b2MouseJoint *joint) { return joint->GetDamping(); }

// --------- b2GearJoint ---------------

b2GearJoint *b2GearJoint_New(b2Joint *joint1, b2Joint *joint2, float ratio, bool collide) {
	b2GearJointDef def;
	def.joint1 = joint1;
	def.joint2 = joint2;
	def.bodyA = joint1->GetBodyB();
	def.bodyB = joint2->GetBodyB();
	def.ratio = ratio;
	def.collideConnected = collide;

	return static_cast<b2GearJoint*>(def.bodyA->GetWorld()->CreateJoint(&def));
}

b2Joint* b2GearJoint_GetJoint1(b2GearJoint *joint) { return joint->GetJoint1(); }
b2Joint* b2GearJoint_GetJoint2(b2GearJoint *joint) { return joint->GetJoint2(); }
void b2GearJoint_SetRatio(b2GearJoint *joint, float ratio) { joint->SetRatio(ratio); }
float b2GearJoint_GetRatio(b2GearJoint *joint) { return joint->GetRatio(); }

// --------- b2WheelJoint ---------------

b2WheelJoint *b2WheelJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, float ax, float ay, bool collide) {
	b2WheelJointDef def;
	def.Initialize(bodyA, bodyB, b2Vec2(x, y), b2Vec2(ax, ay));
	def.collideConnected = collide;
	return static_cast<b2WheelJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

Vector2 *b2WheelJoint_GetLocalAnchorA(b2WheelJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorA(), output);
}

Vector2 *b2WheelJoint_GetLocalAnchorB(b2WheelJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorB(), output);
}

Vector2 *b2WheelJoint_GetLocalAxisA(b2WheelJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAxisA(), output);
}

float b2WheelJoint_GetJointTranslation(b2WheelJoint *joint) { return joint->GetJointTranslation(); }
float b2WheelJoint_GetJointLinearSpeed(b2WheelJoint *joint) { return joint->GetJointLinearSpeed(); }
float b2WheelJoint_GetJointAngle(b2WheelJoint *joint) { return joint->GetJointAngle(); }
float b2WheelJoint_GetJointAngularSpeed(b2WheelJoint *joint) { return joint->GetJointAngularSpeed(); }
bool b2WheelJoint_IsLimitEnabled(b2WheelJoint *joint) { return joint->IsLimitEnabled(); }
void b2WheelJoint_EnableLimit(b2WheelJoint *joint, bool flag) { joint->EnableLimit(flag); }
float b2WheelJoint_GetLowerLimit(b2WheelJoint *joint) { return joint->GetLowerLimit(); }
float b2WheelJoint_GetUpperLimit(b2WheelJoint *joint) { return joint->GetUpperLimit(); }
void b2WheelJoint_SetLimits(b2WheelJoint *joint, float lower, float upper) { joint->SetLimits(lower, upper); }
bool b2WheelJoint_IsMotorEnabled(b2WheelJoint *joint) { return joint->IsMotorEnabled(); }
void b2WheelJoint_EnableMotor(b2WheelJoint *joint, bool flag) { joint->EnableMotor(flag); }
void b2WheelJoint_SetMotorSpeed(b2WheelJoint *joint, float speed) { joint->SetMotorSpeed(speed); }
float b2WheelJoint_GetMotorSpeed(b2WheelJoint *joint) { return joint->GetMotorSpeed(); }
void b2WheelJoint_SetMaxMotorTorque(b2WheelJoint *joint, float torque) { joint->SetMaxMotorTorque(torque); }
float b2WheelJoint_GetMaxMotorTorque(b2WheelJoint *joint) { return joint->GetMaxMotorTorque(); }
float b2WheelJoint_GetMotorTorque(b2WheelJoint *joint, float inv_dt) { return joint->GetMotorTorque(inv_dt); }
void b2WheelJoint_SetStiffness(b2WheelJoint *joint, float stiffness) { joint->SetStiffness(stiffness); }
float b2WheelJoint_GetStiffness(b2WheelJoint *joint) { return joint->GetStiffness(); }
void b2WheelJoint_SetDamping(b2WheelJoint *joint, float damping) { joint->SetDamping(damping); }
float b2WheelJoint_GetDamping(b2WheelJoint *joint) { return joint->GetDamping(); }

// --------- b2WeldJoint ---------------

b2WeldJoint *b2WeldJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, float angle, bool collide) {
	b2WeldJointDef def;
	def.Initialize(bodyA, bodyB, b2Vec2(x, y));
	def.collideConnected = collide;
	def.referenceAngle = angle;
	return static_cast<b2WeldJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

Vector2 *b2WeldJoint_GetLocalAnchorA(b2WeldJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorA(), output);
}

Vector2 *b2WeldJoint_GetLocalAnchorB(b2WeldJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorB(), output);
}

float b2WeldJoint_GetReferenceAngle(b2WeldJoint *joint) { return joint->GetReferenceAngle(); }
void b2WeldJoint_SetStiffness(b2WeldJoint *joint, float stiffness) { joint->SetStiffness(stiffness); }
float b2WeldJoint_GetStiffness(b2WeldJoint *joint) { return joint->GetStiffness(); }
void b2WeldJoint_SetDamping(b2WeldJoint *joint, float damping) { joint->SetDamping(damping); }
float b2WeldJoint_GetDamping(b2WeldJoint *joint) { return joint->GetDamping(); }

// --------- b2FrictionJoint ---------------

b2FrictionJoint *b2FrictionJoint_New(b2Body *bodyA, b2Body *bodyB, float x, float y, bool collide) {
	b2FrictionJointDef def;
	def.Initialize(bodyA, bodyB, b2Vec2(x, y));
	def.collideConnected = collide;
	return static_cast<b2FrictionJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

Vector2 *b2FrictionJoint_GetLocalAnchorA(b2FrictionJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorA(), output);
}

Vector2 *b2FrictionJoint_GetLocalAnchorB(b2FrictionJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLocalAnchorB(), output);
}

void b2FrictionJoint_SetMaxForce(b2FrictionJoint *joint, float force) { joint->SetMaxForce(force); }
float b2FrictionJoint_GetMaxForce(b2FrictionJoint *joint) { return joint->GetMaxForce(); }
void b2FrictionJoint_SetMaxTorque(b2FrictionJoint *joint, float torque) { joint->SetMaxTorque(torque); }
float b2FrictionJoint_GetMaxTorque(b2FrictionJoint *joint) { return joint->GetMaxTorque(); }

// --------- b2MotorJoint ---------------

b2MotorJoint *b2MotorJoint_New(b2Body *bodyA, b2Body *bodyB, bool collide) {
	b2MotorJointDef def;
	def.Initialize(bodyA, bodyB);
	def.collideConnected = collide;
	return static_cast<b2MotorJoint*>(bodyA->GetWorld()->CreateJoint(&def));
}

void b2MotorJoint_SetLinearOffset(b2MotorJoint *joint, float x, float y) { joint->SetLinearOffset(b2Vec2(x, y)); }
Vector2 *b2MotorJoint_GetLinearOffset(b2MotorJoint *joint, Vector2 *output) {
	return convertVector(joint->GetLinearOffset(), output);
}

void b2MotorJoint_SetAngularOffset(b2MotorJoint *joint, float angularOffset) { joint->SetAngularOffset(angularOffset); }
float b2MotorJoint_GetAngularOffset(b2MotorJoint *joint) { return joint->GetAngularOffset(); }
void b2MotorJoint_SetMaxForce(b2MotorJoint *joint, float force) { joint->SetMaxForce(force); }
float b2MotorJoint_GetMaxForce(b2MotorJoint *joint) { return joint->GetMaxForce(); }
void b2MotorJoint_SetMaxTorque(b2MotorJoint *joint, float torque) { joint->SetMaxTorque(torque); }
float b2MotorJoint_GetMaxTorque(b2MotorJoint *joint) { return joint->GetMaxTorque(); }
void b2MotorJoint_SetCorrectionFactor(b2MotorJoint *joint, float factor) { joint->SetCorrectionFactor(factor); }
float b2MotorJoint_GetCorrectionFactor(b2MotorJoint *joint) { return joint->GetCorrectionFactor(); }
