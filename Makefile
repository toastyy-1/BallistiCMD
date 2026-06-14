CXX      := g++
CXXFLAGS := -std=c++20 -Wall -Wpedantic -Isrc -pthread -O3 -flto -funroll-loops

COMMON_SRCS := src/main.cpp src/renderer/renderer.cpp src/renderer/geometry.cpp \
               src/sim/sim.cpp src/sim/rocket.cpp src/fc/fc.cpp

RAYLIB_SRCS := src/renderer/raylib/raylib_backend.cpp src/renderer/raylib/models.cpp
RAYLIB_ARCH := -march=native
TARGET      := program

# --- bgfx backend (`make bgfx`) ---
BGFX_SRCS   := src/renderer/bgfx/bgfx_backend.cpp src/renderer/bgfx/models.cpp
BGFX_ARCH   := -mtune=native
BGFX_TARGET := program-bgfx
BGFX_DIR    := build/bgfx
BGFX_SUB    := third_party/bgfx.cmake
SHADER_DIR  := src/renderer/bgfx/shaders
ASSET_DIR   := src/renderer/bgfx
TEX_ZIP     := $(ASSET_DIR)/bgfx.textures.zip
BGFX_DEFS   := -DUSE_BGFX -D_USE_MATH_DEFINES -DBX_CONFIG_DEBUG=0
BGFX_INCS   := -I$(BGFX_SUB)/bgfx/include -I$(BGFX_SUB)/bx/include -I$(BGFX_SUB)/bimg/include
BGFX_LIBS   := $(BGFX_DIR)/cmake/bgfx/libbgfx.a \
               $(BGFX_DIR)/cmake/bimg/libbimg_decode.a \
               $(BGFX_DIR)/cmake/bimg/libbimg.a \
               $(BGFX_DIR)/cmake/bx/libbx.a

ifeq ($(OS),Windows_NT)
    LDLIBS       := -lraylib -lopengl32 -lgdi32 -lwinmm
    TARGET       := program.exe
    BGFX_TARGET  := program-bgfx.exe
    BGFX_SYSLIBS := -lglfw3 -lopengl32 -lgdi32 -luser32 -lshell32 -lkernel32 -lwinmm
    SHADERC      := $(BGFX_DIR)/cmake/bgfx/shaderc.exe
    CMAKE_GEN    := MinGW Makefiles
    RM           := del /Q
    UNZIP        := tar -xf
    UNZIP_DEST   := -C
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        LDLIBS       := -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
        BGFX_SYSLIBS := -lglfw -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -framework Metal -framework QuartzCore
    else
        LDLIBS       := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
        BGFX_SYSLIBS := -lglfw -lGL -lm -lpthread -ldl -lrt -lX11
    endif
    SHADERC   := $(BGFX_DIR)/cmake/bgfx/shaderc
    CMAKE_GEN := Unix Makefiles
    RM        := rm -f
    UNZIP     := unzip -o
    UNZIP_DEST := -d
endif

# ---------------------------------------------------------------------------
# raylib (default)
# ---------------------------------------------------------------------------
$(TARGET): $(COMMON_SRCS) $(RAYLIB_SRCS)
	$(CXX) $(CXXFLAGS) $(RAYLIB_ARCH) $(COMMON_SRCS) $(RAYLIB_SRCS) -o $@ $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

# ---------------------------------------------------------------------------
# bgfx
# ---------------------------------------------------------------------------
SHADER_SRCS := $(wildcard $(SHADER_DIR)/vs_*.sc) $(wildcard $(SHADER_DIR)/fs_*.sc)
SHADER_BINS := $(SHADER_SRCS:.sc=.bin)

bgfx: $(BGFX_TARGET)

$(BGFX_TARGET): $(COMMON_SRCS) $(BGFX_SRCS) shaders-bgfx
	$(CXX) $(CXXFLAGS) $(BGFX_ARCH) $(BGFX_DEFS) $(BGFX_INCS) \
	    $(COMMON_SRCS) $(BGFX_SRCS) -o $@ $(BGFX_LIBS) $(BGFX_SYSLIBS)

run-bgfx: bgfx
	./$(BGFX_TARGET)

shaders-bgfx: $(SHADER_BINS)

$(SHADER_DIR)/vs_%.bin: $(SHADER_DIR)/vs_%.sc $(SHADER_DIR)/varying.def.sc
	$(SHADERC) -f $< -o $@ --type vertex   --platform linux -p 150 -i $(BGFX_SUB)/bgfx/src --varyingdef $(SHADER_DIR)/varying.def.sc

$(SHADER_DIR)/fs_%.bin: $(SHADER_DIR)/fs_%.sc $(SHADER_DIR)/varying.def.sc
	$(SHADERC) -f $< -o $@ --type fragment --platform linux -p 150 -i $(BGFX_SUB)/bgfx/src --varyingdef $(SHADER_DIR)/varying.def.sc

$(TEX_ZIP):
	git lfs pull --include "$(TEX_ZIP)"
	$(UNZIP) $(TEX_ZIP) $(UNZIP_DEST) $(ASSET_DIR)

bgfx-deps: $(TEX_ZIP)
	git submodule update --init --recursive
	cmake -S $(BGFX_SUB) -B $(BGFX_DIR) -G "$(CMAKE_GEN)" \
	    -DCMAKE_BUILD_TYPE=Release -DBGFX_BUILD_EXAMPLES=OFF -DBGFX_BUILD_TOOLS=ON
	cmake --build $(BGFX_DIR) -j

clean:
	$(RM) $(TARGET) $(BGFX_TARGET)

.PHONY: run run-bgfx bgfx shaders-bgfx bgfx-deps clean