#include "LuaVM.h"
#include <cstring>

extern "C" {
	#include "luajit.h"
	#include "lualib.h"
	#include "lauxlib.h"

	int luaopen_yue(lua_State *L);
	int luaopen_utf8(lua_State *L);
}

static const char BOOT[] = R"code(
package.path = '?.lua;?/init.lua'
yuema = {
	args = { select(2, ...) },
}

do
	local ffi = require('ffi')
	local imports = (...)

	for i = 1, #imports, 4 do
		local name, type, decl, func = imports[i], imports[i + 1], imports[i + 2], imports[i + 3]
		ffi.cdef(decl)
		yuema[name] = ffi.cast(('%s*'):format(type), func)
	end

	local function boot()
		require('lib.core')

		local run = (yuema.args[2] or 'main')
			:gsub('%.[lL][uU][aA]$', '')
			:gsub('%.[yY][uU][eE]$', '')
			:gsub('[/\\]', '.')
		require(run)
	end

	local ok, err = xpcall(boot, function(err)
		return yue.traceback(err)
	end)
	if not ok then error(err, 2) end
end
)code";

void initYue(lua_State *L) {
	luaopen_yue(L);
	lua_getglobal(L, "yue");
	int t = lua_gettop(L);
	lua_getfield(L, t, "insert_loader");
	lua_pcall(L, 0, 0, 0);
	lua_settop(L, t - 1);
}

LuaVM::LuaVM() {
	_state = luaL_newstate();
	luaL_openlibs(_state);
	luaopen_utf8(_state);
	initYue(_state);
}

LuaVM::~LuaVM() {
	lua_close(_state);
}

void LuaVM::run(const FFIExport* exports, int argc, char* argv[]) {
	int err = luaL_loadbuffer(_state, BOOT, strlen(BOOT), "@boot.lua");
	switch(err) {
		case 0: break;
		case LUA_ERRSYNTAX: throw LuaError("Syntax error in boot sequence"); break;
		case LUA_ERRMEM: throw LuaError("Memory allocation error in boot sequence"); break;
		default: throw LuaError("Could not load boot sequence"); break;
	}

	int index = 1;
	lua_newtable(_state);
	for(const FFIExport *exp = exports; exp->name; ++exp) {
	 	lua_pushstring(_state, exp->name);
		lua_rawseti(_state, -2, index++);
		lua_pushstring(_state, exp->type);
		lua_rawseti(_state, -2, index++);
		lua_pushstring(_state, exp->declarations);
		lua_rawseti(_state, -2, index++);
		lua_pushlightuserdata(_state, exp->functions);
		lua_rawseti(_state, -2, index++);
	}
	for(int i = 0; i < argc; ++i)
		lua_pushstring(_state, argv[i]);

	int ret = lua_pcall(_state, 1 + argc, 0, 0);
	if(ret)
		throw LuaError(lua_tostring(_state, -1));
}
