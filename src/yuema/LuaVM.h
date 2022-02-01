#pragma once
#include <stdexcept>

typedef std::runtime_error LuaError;

struct FFIExport {
	const char *name;
	const char *type;
	const char *declarations;
	void *functions;
};

class LuaVM {
public:
	LuaVM();
	~LuaVM();

	void run(const FFIExport* exports, int argc, char* argv[]);
private:
	struct lua_State *_state;
};
