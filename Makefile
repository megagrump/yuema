.PHONY: clean debug release

.ONESHELL:

default: release

build/Makefile:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=${buildmode}

clean:
	rm -rf build
	cd src/LuaJIT && make clean

debug: buildmode:=Debug
debug: build/Makefile
	cmake --build build -j

release: buildmode:=Release
release: build/Makefile
	cmake --build build -j
