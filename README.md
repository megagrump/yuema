# Yuema

Yuema is a Lua-based programming framework combining [raylib](https://github.com/raysan5/raylib) (and some of its accompanying libraries), [Box2D](https://github.com/erincatto/box2d)\*, [LuaJIT](https://github.com/LuaJIT/LuaJIT) and [Yuescript](https://github.com/pigpigyyy/Yuescript) into a stand-alone executable with minimal external dependencies.

You can use it to write 2D and 3D games in Lua or Yuescript (a MoonScript derivative that compiles to Lua). Yuescript integration is seamless with no extra compilation step required.

\* Box2D support is WIP.

## Build

Make sure you clone with submodules:

    git clone --recurse-submodules git@github.com:megagrump/yuema.git

### Linux

```
mkdir build
cd build
cmake ..
make
```

### Other platforms

No one has tried yet. Let me know if you managed to build it on Windows/MacOS/anything!

## Examples

Some examples can be found in the `examples` directory.

```
./build/yuema examples/gui_controls.yue
./build/yuema examples/core_3d_camera_first_person.yue
./build/yuema examples/models_mesh_picking.yue
./build/yuema examples/physics_demo.yue
./build/yuema examples/physics_movement.lua
```

You **must** run the examples from the project root. 

## Code and libraries used in this project

* raylib: https://github.com/raysan5/raylib Copyright (c) 2013-2022 Ramon Santamaria
* raygui: https://github.com/raysan5/raygui Copyright (c) 2014-2022 Ramon Santamaria
* Box2D: https://github.com/erincatto/box2d Copyright (c) 2019 Erin Catto
* physac: https://github.com/victorfisac/Physac Copyright (c) 2016-2020 Victor Fisac 
* LuaJIT: https://github.com/LuaJIT/LuaJIT Copyright (C) 2005-2022 Mike Pall
* Yuescript: https://github.com/pigpigyyy/Yuescript Copyright (c) 2021 Li Jin
* luautf8: https://github.com/starwing/luautf8 Copyright (c) 2018 Xavier Wang
* CParser: https://github.com/facebookresearch/CParser Copyright (c) Facebook, Inc. and its affiliates.
