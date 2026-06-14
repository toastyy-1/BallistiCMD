Missile Targeting Program and Epic 6DoF

Copyright (c) 2026 [Tyler Wiggins]. All rights reserved.

This project and its contents are proprietary and confidential.
No part of this repository may be reproduced, distributed, transmitted,
displayed, or used in any form or by any means without the prior written
permission of the copyright holder.

---

### bgfx backend

```cpp
// in src/main.cpp
#include "renderer/bgfx/bgfx_backend.hpp"   // was: renderer/raylib/raylib_backend.hpp
...
renderer::BgfxBackend backend;              // was: renderer::RaylibBackend backend;
```

**1. Fetch the bgfx submodule** (`third_party/bgfx.cmake`):

```sh
git submodule update --init --recursive
```

**2. Build bgfx/bx/bimg + `shaderc.exe` from the submodule** (one-time):

```sh
cmake -S third_party/bgfx.cmake -B build/bgfx -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Release -DBGFX_BUILD_EXAMPLES=OFF -DBGFX_BUILD_TOOLS=ON
cmake --build build/bgfx -j
```

**3. Compile the shaders with `shaderc`** (forced OpenGL, GLSL 150; run in git-bash):

```sh
SHADERC=build/bgfx/cmake/bgfx/shaderc.exe
SDIR=src/renderer/bgfx/shaders
BGFXSRC=third_party/bgfx.cmake/bgfx/src

for f in "$SDIR"/vs_*.sc; do
  "$SHADERC" -f "$f" -o "${f%.sc}.bin" --type vertex   --platform linux -p 150 \
    -i "$BGFXSRC" --varyingdef "$SDIR/varying.def.sc"
done
for f in "$SDIR"/fs_*.sc; do
  "$SHADERC" -f "$f" -o "${f%.sc}.bin" --type fragment --platform linux -p 150 \
    -i "$BGFXSRC" --varyingdef "$SDIR/varying.def.sc"
done
```

**4. Build the app** (GLFW paths come from Conan — pull the root out of `build/conandeps.mk`):

```sh
GLFW=$(sed -n 's/^CONAN_ROOT_GLFW = //p' build/conandeps.mk)

g++ -std=c++20 -Wall -Wpedantic -Isrc -pthread -O3 -mtune=native -flto -funroll-loops \
  -D_USE_MATH_DEFINES -DBX_CONFIG_DEBUG=0 \
  -Ithird_party/bgfx.cmake/bgfx/include \
  -Ithird_party/bgfx.cmake/bx/include \
  -Ithird_party/bgfx.cmake/bimg/include \
  -I"$GLFW/include" \
  src/main.cpp src/renderer/renderer.cpp src/renderer/geometry.cpp \
  src/sim/sim.cpp src/sim/rocket.cpp src/fc/fc.cpp \
  src/renderer/bgfx/bgfx_backend.cpp src/renderer/bgfx/models.cpp \
  -o program-bgfx.exe \
  build/bgfx/cmake/bgfx/libbgfx.a \
  build/bgfx/cmake/bimg/libbimg_decode.a \
  build/bgfx/cmake/bimg/libbimg.a \
  build/bgfx/cmake/bx/libbx.a \
  -L"$GLFW/lib" -lglfw3 \
  -lopengl32 -lgdi32 -luser32 -lshell32 -lkernel32 -lwinmm
```

**5. Run** with the MinGW toolchain first on `PATH` (so the matching `libstdc++` loads):

```sh
PATH="/c/tools/mingw64/bin:$PATH" ./program-bgfx.exe
```

> Notes: link order matters for the static libs (`bgfx` → `bimg` → `bx`). The bgfx
> Earth maps (`src/renderer/bgfx/*.dds`) and compiled shader `*.bin` are gitignored —
> regenerate the shaders with step 4. `-DBX_CONFIG_DEBUG=0` is required or bx headers
> fail to compile.