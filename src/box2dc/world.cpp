#include "b2_world.h"
#include "b2_world_callbacks.h"
#include "b2_body.h"
#include "box2dc.h"

inline Vector2 *convertVector(const b2Vec2& v, Vector2 *output) {
	output->x = v.x;
	output->y = v.y;
	return output;
}

class RayCaster : public b2RayCastCallback {
public:
	void Set(b2cRayCastCallback callback) { m_callback = callback; }
	b2cRayCastCallback Get() const { return m_callback; }

	float ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) {
		return m_callback(fixture, point.x, point.y, normal.x, normal.y, fraction);
	}
private:
	b2cRayCastCallback m_callback;
};

class AABBQuery : public b2QueryCallback {
public:
	void Set(b2cQueryCallback callback) { m_callback = callback; }
	b2cQueryCallback Get() const { return m_callback; }

	bool ReportFixture(b2Fixture *fixture) {
		return m_callback(fixture);
	}
private:
	b2cQueryCallback m_callback;
};

extern b2ContactFilter b2_defaultFilter;

class ContactFilter : public b2ContactFilter {
public:
	void Set(b2cContactFilter callback) { m_callback = callback; }
	b2cContactFilter Get() const { return m_callback; }

	bool ShouldCollide(b2Fixture *fixtureA, b2Fixture *fixtureB) {
		return m_callback(fixtureA, fixtureB);
	}
private:
	b2cContactFilter m_callback;
};

class ContactListener : public b2ContactListener {
public:
	void Set(b2cContactListener begin, b2cContactListener end, b2cContactListener pre, b2cPostContactListener post) {
		m_begin = begin;
		m_end = end;
		m_preSolve = pre;
		m_postSolve = post;
	}

	void Enable(bool begin, bool end, bool pre, bool post) {
		m_doBegin = begin;
		m_doEnd = end;
		m_doPreSolve = pre;
		m_doPostSolve = post;
	}

	b2cContactListener GetBegin() const { return m_begin; }
	b2cContactListener GetEnd() const { return m_end; }
	b2cContactListener GetPreSolve() const { return m_preSolve; }
	b2cPostContactListener GetPostSolve() const { return m_postSolve; }

	void BeginContact(b2Contact* contact) { if(m_doBegin) m_begin(contact); }
	void EndContact(b2Contact* contact) { if(m_doEnd) m_end(contact); }
	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) { if(m_doPreSolve) m_preSolve(contact); }
	void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) { if(m_doPostSolve) m_postSolve(contact); } // TODO: impulse
private:
	bool m_doBegin, m_doEnd, m_doPreSolve, m_doPostSolve;
	b2cContactListener m_begin, m_end, m_preSolve;
	b2cPostContactListener m_postSolve;
};

extern b2ContactListener b2_defaultListener;

class b2cWorld : public b2World {
public:
	b2cWorld(const b2Vec2 &gravity) : b2World(gravity) {
		b2BodyDef def;
		m_groundBody = CreateBody(&def);
	}

	void RayCast(float x1, float y1, float x2, float y2) {
		static b2Vec2 p1, p2;
		p1.x = x1;
		p1.y = y1;
		p2.x = x2;
		p2.y = y2;
		b2World::RayCast(&m_rayCaster, p1, p2);
	}

	void SetRayCastCallback(b2cRayCastCallback callback) { m_rayCaster.Set(callback); }
	b2cRayCastCallback GetRayCastCallback() const { return m_rayCaster.Get(); }

	void EnableContactFilter(bool flag) {
		if(flag) {
			b2World::SetContactFilter(&m_contactFilter);
		}
		else {
			b2World::SetContactFilter(&b2_defaultFilter);
		}
	}

	void SetContactFilter(b2cContactFilter filter) {
		m_contactFilter.Set(filter);
		if(!filter)
			EnableContactFilter(false);
	}

	b2cContactFilter GetContactFilter() const { return m_contactFilter.Get(); }

	void EnableContactListener(bool begin, bool end, bool pre, bool post) {
		m_contactListener.Enable(begin, end, pre, post);
		if(begin || end || pre || post) {
			b2World::SetContactListener(&m_contactListener);
		}
		else {
			b2World::SetContactListener(nullptr);
		}
	}

	void SetContactListener(b2cContactListener begin, b2cContactListener end, b2cContactListener pre, b2cPostContactListener post) {
		m_contactListener.Set(begin, end, pre, post);
		if(!begin && !end && !pre && !post)
			EnableContactListener(false, false, false, false);
	}

	const ContactListener &GetContactListener() const { return m_contactListener; }
	void SetQueryCallback(b2cQueryCallback callback) { m_query.Set(callback); }
	b2cQueryCallback GetQueryCallback() const { return m_query.Get(); }

	void QueryAABB(float minX, float minY, float maxX, float maxY) {
		static b2AABB aabb;
		aabb.lowerBound.x = minX;
		aabb.lowerBound.y = minY;
		aabb.upperBound.x = maxX;
		aabb.upperBound.y = maxY;
		b2World::QueryAABB(&m_query, aabb);
	}

	b2Body *GetGroundBody() const { return m_groundBody; }
private:
	RayCaster m_rayCaster;
	AABBQuery m_query;
	ContactFilter m_contactFilter;
	ContactListener m_contactListener;
	b2Body *m_groundBody;
};

b2World *b2World_New(float gravityX, float gravityY) {
	b2World *world = new b2cWorld(b2Vec2(gravityX, gravityY));
	return world;
}

void b2World_Destroy(b2World *world) {
	delete static_cast<b2cWorld*>(world);
}

/*void b2World_SetDestructionListener(b2World *world, b2DestructionListener *listener) {
	world->SetDestructionListener(listener);
}*/

void b2World_SetContactFilter(b2World *world, b2cContactFilter filter) {
	static_cast<b2cWorld*>(world)->SetContactFilter(filter);
}

b2cContactFilter b2World_GetContactFilter(b2World *world) {
	return static_cast<b2cWorld*>(world)->GetContactFilter();
}

void b2World_EnableContactFilter(b2World *world, bool flag) {
	static_cast<b2cWorld*>(world)->EnableContactFilter(flag);
}

void b2World_SetContactListener(b2World *world, b2cContactListener begin, b2cContactListener end, b2cContactListener preSolve, b2cPostContactListener postSolve) {
	static_cast<b2cWorld*>(world)->SetContactListener(begin, end, preSolve, postSolve);
}

b2cContactListener b2World_GetBeginContactListener(b2World *world) {
	return static_cast<b2cWorld*>(world)->GetContactListener().GetBegin();
}

b2cContactListener b2World_GetEndContactListener(b2World *world) {
	return static_cast<b2cWorld*>(world)->GetContactListener().GetEnd();
}

b2cContactListener b2World_GetPreSolveContactListener(b2World *world) {
	return static_cast<b2cWorld*>(world)->GetContactListener().GetPreSolve();
}

b2cPostContactListener b2World_GetPostSolveContactListener(b2World *world) {
	return static_cast<b2cWorld*>(world)->GetContactListener().GetPostSolve();
}

void b2World_EnableContactListener(b2World *world, bool begin, bool end, bool pre, bool post) {
	static_cast<b2cWorld*>(world)->EnableContactListener(begin, end, pre, post);
}

void b2World_SetRayCastCallback(b2World *world, b2cRayCastCallback callback) {
	static_cast<b2cWorld*>(world)->SetRayCastCallback(callback);
}

b2cRayCastCallback b2World_GetRayCastCallback(b2World *world) {
	return static_cast<b2cWorld*>(world)->GetRayCastCallback();
}

void b2World_SetQueryCallback(b2World *world, b2cQueryCallback callback) {
	static_cast<b2cWorld*>(world)->SetQueryCallback(callback);
}

b2cQueryCallback b2World_GetQueryCallback(b2World *world) {
	return static_cast<b2cWorld*>(world)->GetQueryCallback();
}

b2Body *b2World_CreateBody(b2World *world, float x, float y, float angle, int type) {
	b2BodyDef def;
	def.position.x = x;
	def.position.y = y;
	def.angle = angle;
	def.type = (b2BodyType)type;
	return world->CreateBody(&def);
}

void b2World_DestroyBody(b2World *world, b2Body *body) { world->DestroyBody(body); }
void b2World_DestroyJoint(b2World *world, b2Joint *joint) { world->DestroyJoint(joint); }
void b2World_Step(b2World *world, float timeStep, int velocityIterations, int positionIterations) {
	world->Step(timeStep, velocityIterations, positionIterations);
}

void b2World_ClearForces(b2World *world) { world->ClearForces(); }
void b2World_DebugDraw(b2World *world) { world->DebugDraw(); }
void b2World_QueryAABB(b2World *world, float minX, float minY, float maxX, float maxY) {
	static_cast<b2cWorld*>(world)->QueryAABB(minX, minY, maxX, maxY);
}

void b2World_RayCast(b2World *world, float x1, float y1, float x2, float y2) {
	static_cast<b2cWorld*>(world)->RayCast(x1, y1, x2, y2);
}

b2Body *b2World_GetBodyList(b2World *world) { return world->GetBodyList(); }
b2Joint *b2World_GetJointList(b2World *world) { return world->GetJointList(); }
b2Contact *b2World_GetContactList(b2World *world) { return world->GetContactList(); }
void b2World_SetAllowSleeping(b2World *world, bool flag) { world->SetAllowSleeping(flag); }
bool b2World_GetAllowSleeping(b2World *world) { return world->GetAllowSleeping(); }
void b2World_SetWarmStarting(b2World *world, bool flag) { world->SetWarmStarting(flag); }
bool b2World_GetWarmStarting(b2World *world) { return world->GetWarmStarting(); }
void b2World_SetContinuousPhysics(b2World *world, bool flag) { world->SetContinuousPhysics(flag); }
bool b2World_GetContinuousPhysics(b2World *world) { return world->GetContinuousPhysics(); }
void b2World_SetSubStepping(b2World *world, bool flag) { world->SetSubStepping(flag); }
bool b2World_GetSubStepping(b2World *world) { return world->GetSubStepping(); }
int b2World_GetProxyCount(b2World *world) { return world->GetProxyCount(); }
int b2World_GetBodyCount(b2World *world) { return world->GetBodyCount(); }
int b2World_GetJointCount(b2World *world) { return world->GetJointCount(); }
int b2World_GetContactCount(b2World *world) { return world->GetContactCount(); }
int b2World_GetTreeHeight(b2World *world) { return world->GetTreeHeight(); }
int b2World_GetTreeBalance(b2World *world) { return world->GetTreeBalance(); }
float b2World_GetTreeQuality(b2World *world) { return world->GetTreeQuality(); }
void b2World_SetGravity(b2World *world, float gravityX, float gravityY) {
	b2Vec2 gravity = b2Vec2(gravityX, gravityY);
	world->SetGravity(gravity);
}

Vector2 *b2World_GetGravity(b2World *world, Vector2 *output) {
	return convertVector(world->GetGravity(), output);
}

bool b2World_IsLocked(b2World *world) { return world->IsLocked(); }
void b2World_SetAutoClearForces(b2World *world, bool flag) { world->SetAutoClearForces(flag); }
bool b2World_GetAutoClearForces(b2World *world) { return world->GetAutoClearForces(); }

void b2World_ShiftOrigin(b2World *world, float x, float y) { world->ShiftOrigin(b2Vec2(x, y)); }

void b2World_Dump(b2World *world) { world->Dump(); }

b2cProfile *b2World_GetProfile(b2World *world, b2cProfile *output) {
	const b2Profile &profile = world->GetProfile();
	output->step = profile.step;
	output->collide = profile.collide;
	output->solve = profile.solve;
	output->solveInit = profile.solveInit;
	output->solveVelocity = profile.solveVelocity;
	output->solvePosition = profile.solvePosition;
	output->broadphase = profile.broadphase;
	output->solveTOI = profile.solveTOI;
	return output;
}

b2Body *b2World_GetGroundBody(b2World *world) { return static_cast<b2cWorld*>(world)->GetGroundBody(); }
