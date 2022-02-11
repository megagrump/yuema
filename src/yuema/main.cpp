#include <cstdbool>
#include <cstdarg>
#include <string>
#include <algorithm>
#include <iostream>
#include "LuaVM.h"

#define RLGL_ENABLE_OPENGL_DEBUG_CONTEXT
#include "raylib.h"
#include "rlgl.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "box2dc.h"

#include "raylib_ffi.h"
#include "rlgl_ffi.h"
#include "raygui_ffi.h"
#include "box2dc_ffi.h"

FFIExport exports[] = {
	{ "RL",   "RAYLIB_API", RAYLIB_FFI, &RAYLIB_EXPORTS },
	{ "RLGL", "RLGL_API",   RLGL_FFI,   &RLGL_EXPORTS   },
	{ "RG",   "RAYGUI_API", RAYGUI_FFI, &RAYGUI_EXPORTS },
	{ "B2D",  "BOX2DC_API", BOX2DC_FFI, &BOX2DC_EXPORTS },
	{ nullptr }
};

void errorHandler(const char* error) {
	if(!IsWindowReady())
		return;

	std::string errorMessage(error);
	errorMessage.erase(std::remove(errorMessage.begin(), errorMessage.end(), '\r'), errorMessage.end());
	while(!WindowShouldClose()) {
		BeginDrawing();
		ClearBackground((Color){ 32, 40, 48, 255 });
		GuiDrawIcon(RAYGUI_ICON_DEMON, 20, 20, 2, RED);
		DrawText("ERROR", 58, 18, 40, RED);
		DrawText(errorMessage.c_str(), 30, 68, 10, RAYWHITE);
		EndDrawing();
		WaitTime(15);
	}
}

int main(int argc, char* argv[]) {
	LuaVM lua;
	try {
		lua.run(exports, argc, argv);
	}
	catch(const LuaError& e) {
		std::cout << "ERROR: " << e.what() << "\n";
		errorHandler(e.what());
		return -1;
	}
	return 0;
}
