#pragma once

inline Vector2 *convertVector(const b2Vec2 &v, Vector2 *output) {
	output->x = v.x;
	output->y = v.y;
	return output;
}

inline b2Vec2 *convertPolygon(const Vector2 *vertices, int count) {
	static b2Vec2 tempPolygon[b2_maxPolygonVertices];
	for(int i = 0; i < count; ++i) {
		tempPolygon[i].x = vertices[i].x;
		tempPolygon[i].y = vertices[i].y;
	}
	return tempPolygon;
}
