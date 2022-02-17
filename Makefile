.PHONY: clean debug release

.ONESHELL:

default: release

build/Makefile:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=${BUILDMODE}

clean:
	rm -rf build
	cd src/LuaJIT && make clean

debug: BUILDMODE:=Debug
debug: build/Makefile
	cmake --build build -j

release: BUILDMODE:=Release
release: build/Makefile
	cmake --build build -j
