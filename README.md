# Yuema

Yuema is a Lua-based programming framework combining [raylib](https://github.com/raysan5/raylib) (and some of its accompanying libraries), [Box2D](https://github.com/erincatto/box2d), [LuaJIT](https://github.com/LuaJIT/LuaJIT) and [Yuescript](https://github.com/pigpigyyy/Yuescript) into a stand-alone executable with minimal external dependencies.

You can use it to write 2D and 3D games in [Lua](https://www.lua.org/) or [Yuescript](http://yuescript.org), a [MoonScript](https://moonscript.org/) derivative that compiles to Lua. Yuescript integration is seamless with no extra compilation step required.

![yuema1](https://user-images.githubusercontent.com/31128870/152654645-fc42539f-b09d-4257-a525-11d3f9058327.png)

![yuema2](https://user-images.githubusercontent.com/31128870/152654647-da90a5d0-3904-466e-b0c0-e11e906c9c14.png)

![yuema3](https://user-images.githubusercontent.com/31128870/152654649-9f07a8c1-67a5-494c-8514-f79bb51ff2be.png)

![yuema_box2d](https://user-images.githubusercontent.com/31128870/153551507-26d94ece-6d52-4757-abeb-7d2d6fcc4ebb.png)

## Work in progress!

The API is not stable and will change frequently.

## How to build

Make sure you clone with submodules:

    git clone --recurse-submodules git@github.com:megagrump/yuema.git

### Linux

#### 1. Install dependencies 

Debian based (Ubuntu, etc.)

    sudo apt install libasound2-dev mesa-common-dev libx11-dev libxrandr-dev libxi-dev xorg-dev libgl1-mesa-dev libglu1-mesa-dev

RedHat based (Fedora, etc.)

    sudo dnf install alsa-lib-devel mesa-libGL-devel libX11-devel libXrandr-devel libXi-devel libXcursor-devel libXinerama-devel

CMake is required also.

#### 2. Build

```
mkdir build
cd build
cmake ..
make
```

### Windows

The [MSYS](https://www.msys2.org/) MinGW compiler suite is used to create binaries for Windows. Additionally, `mingw-w64-x86_64-cmake` is required to build the project.
The `/mingw64/bin` directory is expected to be in your `$PATH`.

```
mkdir build
cd build
cmake -G "MSYS Makefiles" ..
make
```

### Other platforms/compilers

No one has tried yet.

## Examples

Some examples can be found in the `examples` directory.

```
./build/yuema examples/gui_controls.yue
./build/yuema examples/core_3d_camera_first_person.yue
./build/yuema examples/models_mesh_picking.yue
./build/yuema examples/box2d_joints.yue
```

You **must** run the examples from the project root. 

# How to make your own game with yuema

You currently have to manually bootstrap new projects, but it's simple:

1. Build yuema or download a binary release.
2. Create a new directory for your project, for example `/home/alice/myproject` or `C:\projects\myproject`.
3. Copy the yuema executable (`yuema` or `yuema.exe` on Windows) to the project directory and rename it, for example `myproject` (or `myproject.exe`).
4. Copy the `lib` directory to the project directory.
5. Create a `main.lua` or `main.yue` file in the project directory. This is your project's entry point.
```
myproject          <-- project directory
  /lib             <-- library code
  main.yue         <-- code entry point
  myproject(.exe)  <-- executable
```
You can run your project by starting `myproject` from the project directory.

## Code and libraries used in this project

* raylib: https://github.com/raysan5/raylib Copyright (c) 2013-2022 Ramon Santamaria
* raygui: https://github.com/raysan5/raygui Copyright (c) 2014-2022 Ramon Santamaria
* Box2D: https://github.com/erincatto/box2d Copyright (c) 2019 Erin Catto
* LuaJIT: https://github.com/LuaJIT/LuaJIT Copyright (C) 2005-2022 Mike Pall
* Yuescript: https://github.com/pigpigyyy/Yuescript Copyright (c) 2021 Li Jin
* luautf8: https://github.com/starwing/luautf8 Copyright (c) 2018 Xavier Wang
* CParser: https://github.com/facebookresearch/CParser Copyright (c) Facebook, Inc. and its affiliates.
