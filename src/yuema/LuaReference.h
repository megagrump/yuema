#pragma once

extern "C" {
	#include "luajit.h"
}

struct LuaReference {
	LuaReference(lua_State *L, int index);
	~LuaReference();

	void get();

	const int _index;
	lua_State *_state;
};

void register_LuaReference(lua_State *L);
