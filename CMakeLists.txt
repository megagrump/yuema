cmake_minimum_required(VERSION 3.10)

if(DEFINED CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build type")
else()
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
endif()

project(yuema)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED False)

set(YUEMA_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/yuema)

if(WIN32)
	set(LINKER_OPTS "-Wl,-subsystem,windows -static")
	set(EXE_SUFFIX ".exe")
else()
	set(LINKER_OPTS "-ldl")
	set(EXE_SUFFIX "")
endif()

### LuaJIT
set(LUAJIT_PROJECT_DIR ${PROJECT_SOURCE_DIR}/src/LuaJIT)
set(LUAJIT_SOURCE_DIR ${LUAJIT_PROJECT_DIR}/src)

add_custom_target(luajit-bin ALL
	WORKING_DIRECTORY ${LUAJIT_PROJECT_DIR}
	COMMAND make BUILDMODE=static CFLAGS=-DLUAJIT_ENABLE_LUA52COMPAT=1 -j
)
add_library(luajit STATIC IMPORTED)
add_dependencies(luajit luajit-bin)
set_property(TARGET luajit PROPERTY IMPORTED_LOCATION ${LUAJIT_SOURCE_DIR}/libluajit.a)
###

function(generate_ffi source target prefix defines)
	add_custom_command(
		OUTPUT ${target}
		COMMAND ${LUAJIT_SOURCE_DIR}/luajit${EXE_SUFFIX} generate_ffi.lua ${source} ${target} ${prefix} ${defines}
		DEPENDS ${source}
		DEPENDS luajit
		DEPENDS ${YUEMA_SOURCE_DIR}/tools/generate_ffi.lua
		WORKING_DIRECTORY ${YUEMA_SOURCE_DIR}/tools
	)
endfunction()

### raylib
set(RAYLIB_PROJECT_DIR ${PROJECT_SOURCE_DIR}/src/raylib)
set(RAYLIB_SOURCE_DIR ${RAYLIB_PROJECT_DIR}/src)
set(CUSTOMIZE_BUILD ON CACHE BOOL "Customize raylib build")
set(SUPPORT_FILEFORMAT_JPG ON CACHE BOOL "Enable JPG support")
#set(GRAPHICS GRAPHICS_API_OPENGL_43 CACHE STRING "OpenGL version")
add_subdirectory(${RAYLIB_PROJECT_DIR})
generate_ffi(${RAYLIB_SOURCE_DIR}/raylib.h ${YUEMA_SOURCE_DIR}/raylib_ffi.h RAYLIB "")
generate_ffi(${RAYLIB_SOURCE_DIR}/rlgl.h ${YUEMA_SOURCE_DIR}/rlgl_ffi.h RLGL "RL_MATRIX_TYPE RLGL_ENABLE_OPENGL_DEBUG_CONTEXT")
###

### raygui
set(RAYGUI_PROJECT_DIR ${PROJECT_SOURCE_DIR}/src/raygui)
set(RAYGUI_SOURCE_DIR ${RAYGUI_PROJECT_DIR}/src)
generate_ffi(${RAYGUI_SOURCE_DIR}/raygui.h ${YUEMA_SOURCE_DIR}/raygui_ffi.h RAYGUI "")
###

### box2d
set(BOX2D_PROJECT_DIR ${PROJECT_SOURCE_DIR}/src/box2d)
set(BOX2D_SOURCE_DIR ${BOX2D_PROJECT_DIR}/src)
set(BOX2D_INCLUDE_DIR ${BOX2D_PROJECT_DIR}/include/box2d)
option(BOX2D_BUILD_UNIT_TESTS OFF)
option(BOX2D_BUILD_TESTBED OFF)
add_subdirectory(${BOX2D_PROJECT_DIR})
###

### box2dc
set(BOX2DC_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/box2dc)
add_library(box2dc OBJECT
	${BOX2DC_SOURCE_DIR}/box2dc.cpp
	${BOX2DC_SOURCE_DIR}/world.cpp
	${BOX2DC_SOURCE_DIR}/shapes.cpp
	${BOX2DC_SOURCE_DIR}/joints.cpp
	${BOX2DC_SOURCE_DIR}/contact.cpp
)
add_dependencies(box2dc box2d)
target_include_directories(box2dc PRIVATE ${BOX2D_INCLUDE_DIR} ${RAYLIB_SOURCE_DIR})
generate_ffi(${BOX2DC_SOURCE_DIR}/box2dc.h ${YUEMA_SOURCE_DIR}/box2dc_ffi.h BOX2DC "")
###

### Yuescript
set(YUE_SOURCE_DIR src/Yuescript/src)

add_library(yue STATIC
	${YUE_SOURCE_DIR}/yuescript/ast.cpp
	${YUE_SOURCE_DIR}/yuescript/parser.cpp
	${YUE_SOURCE_DIR}/yuescript/yue_parser.cpp
	${YUE_SOURCE_DIR}/yuescript/yue_compiler.cpp
	${YUE_SOURCE_DIR}/yuescript/yuescript.cpp
)
add_dependencies(yue luajit)
target_include_directories(yue PRIVATE ${YUE_SOURCE_DIR} ${LUAJIT_SOURCE_DIR})
###

add_executable(yuema
	${PROJECT_SOURCE_DIR}/src/luautf8/lutf8lib.c
	${YUEMA_SOURCE_DIR}/raylib_ffi.h
	${YUEMA_SOURCE_DIR}/raygui_ffi.h
	${YUEMA_SOURCE_DIR}/box2dc_ffi.h
	${YUEMA_SOURCE_DIR}/rlgl_ffi.h
	${YUEMA_SOURCE_DIR}/LuaVM.cpp
	${YUEMA_SOURCE_DIR}/main.cpp
	$<TARGET_OBJECTS:box2dc>
)

target_include_directories(yuema PRIVATE ${RAYLIB_SOURCE_DIR} ${RAYGUI_SOURCE_DIR} ${LUAJIT_SOURCE_DIR} ${BOX2D_INCLUDE_DIR} ${BOX2DC_SOURCE_DIR})
set_target_properties(yuema PROPERTIES LINK_FLAGS_RELEASE -s)
target_link_libraries(yuema ${LINKER_OPTS} -lpthread raylib luajit box2d yue)
