#include <cstdbool>
#include <cstdarg>
#include <iostream>
#include "LuaVM.h"

#define GRAPHICS_API_OPENGL_43
#define RLGL_ENABLE_OPENGL_DEBUG_CONTEXT
#include "raylib.h"
#include "rlgl.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#define PHYSAC_IMPLEMENTATION
#include "physac.h"

#include "raylib_ffi.h"
#include "rlgl_ffi.h"
#include "raygui_ffi.h"
#include "physac_ffi.h"

FFIExport exports[] = {
	{ "RL",   "RAYLIB_API", RAYLIB_FFI, &RAYLIB_EXPORTS },
	{ "RLGL", "RLGL_API",   RLGL_FFI,   &RLGL_EXPORTS   },
	{ "RG",   "RAYGUI_API", RAYGUI_FFI, &RAYGUI_EXPORTS },
	{ "PS",   "PHYSAC_API", PHYSAC_FFI, &PHYSAC_EXPORTS },
	{ nullptr }
};

int main(int argc, char* argv[]) {
	LuaVM lua;
	try {
		lua.run(exports, argc, argv);
	}
	catch(const LuaError& e) {
		std::cout << "ERROR: " << e.what() << "\n";
		return -1;
	}
	return 0;
}
