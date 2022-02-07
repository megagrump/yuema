#include "b2_world.h"
#include "b2_fixture.h"
#include "b2_body.h"
#include "b2_collision.h"
#include "b2_draw.h"
#include "b2_world_callbacks.h"
#include "box2dc.h"

inline Vector2 *convertVector(const b2Vec2& v, Vector2 *output) {
	output->x = v.x;
	output->y = v.y;
	return output;
}

// --------- b2Body ---------------

b2Fixture *b2Body_CreateFixture(b2Body *body, b2Shape *shape, float density) { return body->CreateFixture(shape, density); }
void b2Body_DestroyFixture(b2Body *body, b2Fixture *fixture) { body->DestroyFixture(fixture); }
void b2Body_SetTransform(b2Body *body, float tx, float ty, float angle) { body->SetTransform(b2Vec2(tx, ty), angle); }

Vector2 *b2Body_GetPosition(b2Body *body, Vector2 *output) {
	return convertVector(body->GetPosition(), output);
}

float b2Body_GetAngle(b2Body *body) { return body->GetAngle(); }

Vector2 *b2Body_GetWorldCenter(b2Body *body, Vector2 *output) {
	return convertVector(body->GetWorldCenter(), output);
}

Vector2 *b2Body_GetLocalCenter(b2Body *body, Vector2 *output) {
	return convertVector(body->GetLocalCenter(), output);
}

void b2Body_SetLinearVelocity(b2Body *body, float x, float y) {
	body->SetLinearVelocity(b2Vec2(x, y));
}

Vector2 *b2Body_GetLinearVelocity(b2Body *body, Vector2 *output) {
	return convertVector(body->GetLinearVelocity(), output);
}

void b2Body_SetAngularVelocity(b2Body *body, float omega) {
	body->SetAngularVelocity(omega);
}

float b2Body_GetAngularVelocity(b2Body *body) { return body->GetAngularVelocity(); }

void b2Body_ApplyForce(b2Body *body, float forceX, float forceY, float x, float y, bool wake) {
	body->ApplyForce(b2Vec2(forceX, forceY), b2Vec2(x, y), wake);
}

void b2Body_ApplyForceToCenter(b2Body *body, float forceX, float forceY, bool wake) {
	body->ApplyForceToCenter(b2Vec2(forceX, forceY), wake);
}

void b2Body_ApplyTorque(b2Body *body, float torque, bool wake) { body->ApplyTorque(torque, wake); }

void b2Body_ApplyLinearImpulse(b2Body *body, float impulseX, float impulseY, float x, float y, bool wake) {
	body->ApplyLinearImpulse(b2Vec2(impulseX, impulseY), b2Vec2(x, y), wake);
}

void b2Body_ApplyLinearImpulseToCenter(b2Body *body, float impulseX, float impulseY, bool wake) {
	body->ApplyLinearImpulseToCenter(b2Vec2(impulseX, impulseY), wake);
}

void b2Body_ApplyAngularImpulse(b2Body *body, float impulse, bool wake) { body->ApplyAngularImpulse(impulse, wake); }
float b2Body_GetMass(b2Body *body) { return body->GetMass(); }
float b2Body_GetInertia(b2Body *body) { return body->GetInertia(); }

b2cMassData *b2Body_GetMassData(b2Body *body, b2cMassData *output) {
	b2MassData data = body->GetMassData();
	output->x = data.center.x;
	output->y = data.center.y;
	output->mass = data.mass;
	output->inertia = data.I;
	return output;
}

void b2Body_SetMassData(b2Body *body, float x, float y, float mass, float inertia) {
	b2MassData data;
	data.mass = mass;
	data.center.x = x;
	data.center.y = y;
	data.I = inertia;
	body->SetMassData(&data);
}

void b2Body_ResetMassData(b2Body *body) { body->ResetMassData(); }

Vector2 *b2Body_GetWorldPoint(b2Body *body, float localX, float localY, Vector2 *output) {
	return convertVector(body->GetWorldPoint(b2Vec2(localX, localY)), output);
}

Vector2 *b2Body_GetWorldVector(b2Body *body, float localX, float localY, Vector2 *output) {
	return convertVector(body->GetWorldVector(b2Vec2(localX, localY)), output);
}

Vector2 *b2Body_GetLocalPoint(b2Body *body, float worldX, float worldY, Vector2 *output) {
	return convertVector(body->GetLocalPoint(b2Vec2(worldX, worldY)), output);
}

Vector2 *b2Body_GetLocalVector(b2Body *body, float worldX, float worldY, Vector2 *output) {
	return convertVector(body->GetLocalVector(b2Vec2(worldX, worldY)), output);
}

Vector2 *b2Body_GetLinearVelocityFromWorldPoint(b2Body *body, float worldX, float worldY, Vector2 *output) {
	return convertVector(body->GetLinearVelocityFromWorldPoint(b2Vec2(worldX, worldY)), output);
}

Vector2 *b2Body_GetLinearVelocityFromLocalPoint(b2Body *body, float localX, float localY, Vector2 *output) {
	return convertVector(body->GetLinearVelocityFromLocalPoint(b2Vec2(localX, localY)), output);
}

float b2Body_GetLinearDamping(b2Body *body) { return body->GetLinearDamping(); }
void b2Body_SetLinearDamping(b2Body *body, float linearDamping) { body->SetLinearDamping(linearDamping); }
float b2Body_GetAngularDamping(b2Body *body) { return body->GetAngularDamping(); }
void b2Body_SetAngularDamping(b2Body *body, float angularDamping) { body->SetAngularDamping(angularDamping); }
float b2Body_GetGravityScale(b2Body *body) { return body->GetGravityScale(); }
void b2Body_SetGravityScale(b2Body *body, float scale) { body->SetGravityScale(scale); }
void b2Body_SetType(b2Body *body, int type) { body->SetType((b2BodyType)type); }
int b2Body_GetType(b2Body *body) { return body->GetType(); }
void b2Body_SetBullet(b2Body *body, bool flag) { body->SetBullet(flag); }
bool b2Body_IsBullet(b2Body *body) { return body->IsBullet(); }
void b2Body_SetSleepingAllowed(b2Body *body, bool flag) { body->SetSleepingAllowed(flag); }
bool b2Body_IsSleepingAllowed(b2Body *body) { return body->IsSleepingAllowed(); }
void b2Body_SetAwake(b2Body *body, bool flag) { body->SetAwake(flag); }
bool b2Body_IsAwake(b2Body *body) { return body->IsAwake(); }
void b2Body_SetEnabled(b2Body *body, bool flag) { body->SetEnabled(flag); }
bool b2Body_IsEnabled(b2Body *body) { return body->IsEnabled(); }
void b2Body_SetFixedRotation(b2Body *body, bool flag) { body->SetFixedRotation(flag); }
bool b2Body_IsFixedRotation(b2Body *body) { return body->IsFixedRotation(); }
b2Fixture *b2Body_GetFixtureList(b2Body *body) { return body->GetFixtureList(); }
//b2JointEdge *b2Body_GetJointList(b2Body *body) { return body->GetJointList(); }
//b2ContactEdge *b2Body_GetContactList(b2Body *body) { return body->GetContactList(); }
b2Body *b2Body_GetNext(b2Body *body) { return body->GetNext(); }
uintptr_t b2Body_GetUserData(b2Body *body) { return body->GetUserData().pointer; }
void b2Body_SetUserData(b2Body *body, uintptr_t data) { body->GetUserData().pointer = data; }
b2World *b2Body_GetWorld(b2Body *body) { return body->GetWorld(); }
void b2Body_Dump(b2Body *body) { body->Dump(); }

// --------- b2Fixture ---------------

int b2Fixture_GetType(b2Fixture *fixture) { return fixture->GetType(); }
b2Shape *b2Fixture_GetShape(b2Fixture *fixture) { return fixture->GetShape(); }
void b2Fixture_SetSensor(b2Fixture *fixture, bool sensor) { fixture->SetSensor(sensor); }
bool b2Fixture_IsSensor(b2Fixture *fixture) { return fixture->IsSensor(); }
void b2Fixture_SetFilterData(b2Fixture *fixture, uint16_t category, uint16_t mask, int16_t group) {
	b2Filter filter;
	filter.categoryBits = category;
	filter.maskBits = mask;
	filter.groupIndex = group;
	fixture->SetFilterData(filter);
}

b2cFilter *b2Fixture_GetFilterData(b2Fixture *fixture, b2cFilter *output) {
	const b2Filter &filter = fixture->GetFilterData();
	output->category = filter.categoryBits;
	output->mask = filter.maskBits;
	output->group = filter.groupIndex;
	return output;
}

void b2Fixture_Refilter(b2Fixture *fixture) { fixture->Refilter(); }
b2Body *b2Fixture_GetBody(b2Fixture *fixture) { return fixture->GetBody(); }
b2Fixture *b2Fixture_GetNext(b2Fixture *fixture) { return fixture->GetNext(); }
uintptr_t b2Fixture_GetUserData(b2Fixture *fixture) { return fixture->GetUserData().pointer; }
void b2Fixture_SetUserData(b2Fixture *fixture, uintptr_t data) { fixture->GetUserData().pointer = data; }

bool b2Fixture_TestPoint(b2Fixture *fixture, float x, float y) { return fixture->TestPoint(b2Vec2(x, y)); }
b2cRayCastOutput *b2Fixture_RayCast(b2Fixture *fixture, float x1, float y1, float x2, float y2,
	float maxFraction, int childIndex, b2cRayCastOutput *output) {
	b2RayCastInput ray;
	ray.p1.x = x1;
	ray.p1.y = y1;
	ray.p2.x = x2;
	ray.p2.y = y2;
	ray.maxFraction = maxFraction;

	b2RayCastOutput cast;
	bool hit = fixture->RayCast(&cast, ray, childIndex);
	output->hit = hit;
	output->nx = cast.normal.x;
	output->ny = cast.normal.y;
	output->fraction = cast.fraction;
	return output;
}

b2cMassData *b2Fixture_GetMassData(b2Fixture *fixture, b2cMassData *output) {
	b2MassData data;
	fixture->GetMassData(&data);
	output->x = data.center.x;
	output->y = data.center.y;
	output->mass = data.mass;
	output->inertia = data.I;
	return output;
}

void b2Fixture_SetDensity(b2Fixture *fixture, float density) { fixture->SetDensity(density); }
float b2Fixture_GetDensity(b2Fixture *fixture) { return fixture->GetDensity(); }
float b2Fixture_GetFriction(b2Fixture *fixture) { return fixture->GetFriction(); }
void b2Fixture_SetFriction(b2Fixture *fixture, float friction) { fixture->SetFriction(friction); }
float b2Fixture_GetRestitution(b2Fixture *fixture) { return fixture->GetRestitution(); }
void b2Fixture_SetRestitution(b2Fixture *fixture, float restitution) { fixture->SetRestitution(restitution); }
float b2Fixture_GetRestitutionThreshold(b2Fixture *fixture) { return fixture->GetRestitutionThreshold(); }
void b2Fixture_SetRestitutionThreshold(b2Fixture *fixture, float threshold) { fixture->SetRestitutionThreshold(threshold); }
Rectangle *b2Fixture_GetAABB(b2Fixture *fixture, int childIndex, Rectangle *output) {
	const b2AABB &aabb = fixture->GetAABB(childIndex);
	float w = aabb.upperBound.x - aabb.lowerBound.x;
	float h = aabb.upperBound.y - aabb.lowerBound.y;
	output->x = aabb.lowerBound.x;
	output->y = aabb.lowerBound.y;
	output->width = w;
	output->height = h;
	return output;
}

void b2Fixture_Dump(b2Fixture *fixture, int bodyIndex) { fixture->Dump(bodyIndex); }

// --------- b2Draw ---------------

class DrawProxy : public b2Draw {
public:
	void setCallbacks(b2cDraw *callbacks) { m_callbacks = callbacks; }

	void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override {
		for(int i = 0; i < vertexCount; ++i) {
			m_vertices[i].x = vertices[i].x;
			m_vertices[i].y = vertices[i].y;
		}
		m_callbacks->DrawPolygon(m_vertices, vertexCount, color.r, color.g, color.b, color.a);
	}

	void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override {
		for(int i = 0; i < vertexCount; ++i) {
			m_vertices[i].x = vertices[i].x;
			m_vertices[i].y = vertices[i].y;
		}
		m_callbacks->DrawSolidPolygon(m_vertices, vertexCount, color.r, color.g, color.b, color.a);
	}

	void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) override {
		m_callbacks->DrawCircle(center.x, center.y, radius, color.r, color.g, color.b, color.a);
	}

	void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) override {
		m_callbacks->DrawSolidCircle(center.x, center.y, radius, axis.x, axis.y, color.r, color.g, color.b, color.a);
	}

	void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override {
		m_callbacks->DrawSegment(p1.x, p1.y, p2.x, p2.y, color.r, color.g, color.b, color.a);
	}

	void DrawTransform(const b2Transform& xf) override {
		m_callbacks->DrawTransform(xf.p.x, xf.p.y, xf.q.GetAngle());
	}

	void DrawPoint(const b2Vec2& p, float size, const b2Color& color) override {
		m_callbacks->DrawPoint(p.x, p.y, size, color.r, color.g, color.b, color.a);
	}
private:
	b2cDraw *m_callbacks;
	Vector2 m_vertices[b2_maxPolygonVertices];
};

static DrawProxy proxy;

void b2Draw_Draw(b2World *world, b2cDraw *draw) {
	proxy.SetFlags(0xffff);
	proxy.setCallbacks(draw);
	world->SetDebugDraw(&proxy);
	world->DebugDraw();
	world->SetDebugDraw(nullptr);
}
