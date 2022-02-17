#include "b2_circle_shape.h"
#include "b2_chain_shape.h"
#include "b2_edge_shape.h"
#include "b2_polygon_shape.h"
#include "box2dc.h"
#include "util.h"

b2cMassData *b2Shape_ComputeMass(b2Shape *shape, float density, b2cMassData *output) {
	b2MassData data;
	shape->ComputeMass(&data, density);
	output->x = data.center.x;
	output->y = data.center.y;
	output->mass = data.mass;
	output->inertia = data.I;
	return output;
}

Rectangle *b2Shape_ComputeAABB(b2Shape *shape, float tx, float ty, float angle, int childIndex, Rectangle *output) {
	b2Transform t(b2Vec2(tx, ty), b2Rot(angle));
	b2AABB box;
	shape->ComputeAABB(&box, t, childIndex);
	output->x = box.lowerBound.x;
	output->y = box.lowerBound.y;
	output->width = box.upperBound.x - box.lowerBound.x;
	output->height = box.upperBound.y - box.lowerBound.y;
	return output;
}

void b2Shape_Destroy(b2Shape *shape) { delete shape; }
int b2Shape_GetType(b2Shape *shape) { return shape->m_type; }
int b2Shape_GetChildCount(b2Shape *shape) { return shape->GetChildCount(); }
bool b2Shape_TestPoint(b2Shape *shape, float tx, float ty, float angle, float x, float y) {
	b2Transform t(b2Vec2(tx, ty), b2Rot(angle));
	b2Vec2 p = b2Vec2(x, y);
	return shape->TestPoint(t, p);
}

b2cRayCastOutput *b2Shape_RayCast(b2Shape *shape, float tx, float ty, float angle, float x1, float y1,
	float x2, float y2, float maxFraction, int childIndex, b2cRayCastOutput *output) {
	b2RayCastInput ray;
	ray.p1.x = x1;
	ray.p1.y = y1;
	ray.p2.x = x2;
	ray.p2.y = y2;
	ray.maxFraction = maxFraction;
	b2Transform t(b2Vec2(tx, ty), b2Rot(angle));

	b2RayCastOutput cast;
	bool hit = shape->RayCast(&cast, ray, t, childIndex);
	output->hit = hit;
	output->nx = cast.normal.x;
	output->ny = cast.normal.y;
	output->fraction = cast.fraction;
	return output;
}

// --------- b2CircleShape ---------------

b2CircleShape *b2CircleShape_New(float radius) {
	b2CircleShape *shape = new b2CircleShape();
	shape->m_p = b2Vec2(0, 0);
	shape->m_radius = radius;
	return shape;
}

void b2CircleShape_SetPosition(b2CircleShape *shape, float x, float y) { shape->m_p = b2Vec2(x, y); }
Vector2 *b2CircleShape_GetPosition(b2CircleShape *shape, Vector2 *output) { return convertVector(shape->m_p, output); }
void b2CircleShape_SetRadius(b2CircleShape *shape, float radius) { shape->m_radius = radius; }
float b2CircleShape_GetRadius(b2CircleShape *shape) { return shape->m_radius; }

// --------- b2EdgeShape ---------------

b2EdgeShape *b2EdgeShape_NewTwoSided(float x1, float y1, float x2, float y2) {
	b2EdgeShape *shape = new b2EdgeShape();
	shape->SetTwoSided(b2Vec2(x1, y1), b2Vec2(x2, y2));
	return shape;
}

b2EdgeShape *b2EdgeShape_NewOneSided(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3) {
	b2EdgeShape *shape = new b2EdgeShape();
	shape->SetOneSided(b2Vec2(x0, y0), b2Vec2(x1, y1), b2Vec2(x2, y2), b2Vec2(x3, y3));
	return shape;
}

void b2EdgeShape_SetOneSided(b2EdgeShape *shape, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3) {
	shape->SetOneSided(b2Vec2(x0, x0), b2Vec2(x1, y1), b2Vec2(x2, y2), b2Vec2(x3, y3));
}

void b2EdgeShape_SetTwoSided(b2EdgeShape *shape, float x1, float y1, float x2, float y2) {
	shape->SetTwoSided(b2Vec2(x1, y1), b2Vec2(x2, y2));
}

Vector2 *b2EdgeShape_GetVertex(b2EdgeShape *shape, int index, Vector2 *output) {
	switch(index) {
		case 0:
			output->x = shape->m_vertex0.x;
			output->y = shape->m_vertex0.y;
			break;
		case 1:
			output->x = shape->m_vertex1.x;
			output->y = shape->m_vertex1.y;
			break;
		case 2:
			output->x = shape->m_vertex2.x;
			output->y = shape->m_vertex2.y;
			break;
		case 3:
			output->x = shape->m_vertex3.x;
			output->y = shape->m_vertex3.y;
			break;
	}
	return output;
}

bool b2EdgeShape_IsOneSided(b2EdgeShape *shape) { return shape->m_oneSided; }

// --------- b2PolygonShape ---------------

b2PolygonShape *b2PolygonShape_New(Vector2 *vertices, int count) {
	b2PolygonShape *shape = new b2PolygonShape();
	shape->Set(convertPolygon(vertices, count), count);
	return shape;
}

bool b2PolygonShape_Validate(b2PolygonShape *shape) { return shape->Validate(); }

int b2PolygonShape_GetVertexCount(b2PolygonShape *shape) { return shape->m_count; }

Vector2 *b2PolygonShape_GetVertex(b2PolygonShape *shape, int index, Vector2 *output) {
	return convertVector(shape->m_vertices[index], output);
}

Vector2 *b2PolygonShape_GetNormal(b2PolygonShape *shape, int index, Vector2 *output) {
	return convertVector(shape->m_normals[index], output);
}

// --------- b2ChainShape ---------------

b2ChainShape *b2ChainShape_NewLoop(Vector2 *vertices, int count) {
	b2ChainShape *shape = new b2ChainShape();
	shape->CreateLoop(convertPolygon(vertices, count), count);
	return shape;
}

b2ChainShape *b2ChainShape_NewChain(Vector2 *vertices, int count, float x0, float y0, float x1, float y1) {
	b2ChainShape *shape = new b2ChainShape();
	shape->CreateChain(convertPolygon(vertices, count), count, b2Vec2(x0, y0), b2Vec2(x1, y1));
	return shape;
}

int b2ChainShape_GetVertexCount(b2ChainShape *shape) { return shape->m_count; }

Vector2 *b2ChainShape_GetVertex(b2ChainShape *shape, int index, Vector2 *output) {
	return convertVector(shape->m_vertices[index], output);
}

void b2ChainShape_SetPrevVertex(b2ChainShape *shape, float x, float y) {
	shape->m_prevVertex.x = x;
	shape->m_prevVertex.y = y;
}

Vector2 *b2ChainShape_GetPrevVertex(b2ChainShape *shape, Vector2 *output) {
	return convertVector(shape->m_prevVertex, output);
}

void b2ChainShape_SetNextVertex(b2ChainShape *shape, float x, float y) {
	shape->m_nextVertex.x = x;
	shape->m_nextVertex.y = y;
}

Vector2 *b2ChainShape_GetNextVertex(b2ChainShape *shape, Vector2 *output) {
	return convertVector(shape->m_nextVertex, output);
}
