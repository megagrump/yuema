#include "b2_fixture.h"
#include "b2_contact.h"
#include "box2dc.h"
#include "util.h"

static b2WorldManifold worldManifold;

bool b2Contact_IsTouching(b2Contact *contact) { return contact->IsTouching(); }
void b2Contact_SetEnabled(b2Contact *contact, bool flag) { contact->SetEnabled(flag); }
bool b2Contact_IsEnabled(b2Contact *contact) { return contact->IsEnabled(); }
b2Contact *b2Contact_GetNext(b2Contact *contact) { return contact->GetNext(); }
b2Fixture *b2Contact_GetFixtureA(b2Contact *contact) { return contact->GetFixtureA(); }
int b2Contact_GetChildIndexA(b2Contact *contact) { return contact->GetChildIndexA(); }
b2Fixture *b2Contact_GetFixtureB(b2Contact *contact) { return contact->GetFixtureB(); }
int b2Contact_GetChildIndexB(b2Contact *contact) { return contact->GetChildIndexB(); }
void b2Contact_SetFriction(b2Contact *contact, float friction) { contact->SetFriction(friction); }
float b2Contact_GetFriction(b2Contact *contact) { return contact->GetFriction(); }
void b2Contact_ResetFriction(b2Contact *contact) { contact->ResetFriction(); }
void b2Contact_SetRestitution(b2Contact *contact, float restitution) { contact->SetRestitution(restitution); }
float b2Contact_GetRestitution(b2Contact *contact) { return contact->GetRestitution(); }
void b2Contact_ResetRestitution(b2Contact *contact) { contact->ResetRestitution(); }
void b2Contact_SetRestitutionThreshold(b2Contact *contact, float threshold) { contact->SetRestitutionThreshold(threshold); }
float b2Contact_GetRestitutionThreshold(b2Contact *contact) { return contact->GetRestitutionThreshold(); }
void b2Contact_ResetRestitutionThreshold(b2Contact *contact) { contact->ResetRestitutionThreshold(); }
void b2Contact_SetTangentSpeed(b2Contact *contact, float speed) { contact->SetTangentSpeed(speed); }
float b2Contact_GetTangentSpeed(b2Contact *contact) { return contact->GetTangentSpeed(); }

Vector2 *b2Contact_GetNormal(b2Contact *contact, Vector2* output) {
	contact->GetWorldManifold(&worldManifold);
	return convertVector(worldManifold.normal, output);
}

int b2Contact_GetPoints(b2Contact *contact, Vector2 *output) {
	contact->GetWorldManifold(&worldManifold);
	int n = contact->GetManifold()->pointCount;
	for(int i = 0; i < n; ++i) {
		output[i].x = worldManifold.points[i].x;
		output[i].y = worldManifold.points[i].y;
	}

	return n;
}
