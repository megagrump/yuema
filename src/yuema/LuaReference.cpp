extern "C" {
	#include "lauxlib.h"
}
#include "LuaReference.h"

static const char REF_STORE[] = "yuema-refstore";

void getStore(lua_State *L) {
	lua_getfield(L, LUA_REGISTRYINDEX, REF_STORE);
	if(!lua_isnil(L, -1))
		return;

	lua_pop(L, 1);
	lua_newtable(L);
	lua_pushvalue(L, -1);
	lua_setfield(L, LUA_REGISTRYINDEX, REF_STORE);
}

int storeValue(lua_State *L) {
	if(lua_isnil(L, -1))
		return LUA_REFNIL;

	getStore(L);
	lua_insert(L, -2);
	int index = luaL_ref(L, -2);
	lua_pop(L, 1);
	return index;
}

void unref(lua_State *L, int index) {
	if(index == LUA_REFNIL)
		return;

	getStore(L);
	luaL_unref(L, -1, index);
	lua_pop(L, 1);
}

int ref_create(lua_State *L) {
	int index = storeValue(L);
	if(index != LUA_REFNIL) {
		// TODO: make it full userdata with GC support
		lua_pushlightuserdata(L, new LuaReference(L, index));
	}
	else
		lua_pushnil(L);
	return 1;
}

int ref_release(lua_State *L) {
	LuaReference *ref = static_cast<LuaReference*>(lua_touserdata(L, 1));
	delete ref;
	return 0;
}

int ref_get(lua_State *L) {
	LuaReference *ref = static_cast<LuaReference*>(lua_touserdata(L, 1));
	if(ref) {
		ref->get();
	}
	else
		lua_pushnil(L);
	return 1;
}

LuaReference::LuaReference(lua_State *L, int index = LUA_REFNIL)
	: _state(L)
	, _index(index)
{
}

LuaReference::~LuaReference() {
	unref(_state, _index);
}

void LuaReference::get() {
	if(_index != LUA_REFNIL) {
		getStore(_state);
		lua_rawgeti(_state, -1, _index);
		lua_remove(_state, -2);
	}
	else {
		lua_pushnil(_state);
	}
}

void register_LuaReference(lua_State *L) {
	static luaL_Reg ref[] = {
		{ "create", ref_create },
		{ "release", ref_release },
		{ "get", ref_get },
		{ nullptr, nullptr }
	};

	lua_getglobal(L, "package");
	lua_getfield(L, -1, "loaded");

	lua_newtable(L);
	luaL_register(L, nullptr, ref);
	lua_setfield(L, -2, "yuema.ref");
}
