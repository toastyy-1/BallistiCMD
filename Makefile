CXX      := g++
CXXFLAGS := -std=c++20 -Wall -Wpedantic -Isrc -pthread -O3 -march=native -flto -funroll-loops
SRCS     := src/main.cpp src/renderer/renderer.cpp src/sim/sim.cpp src/sim/rocket.cpp src/fc/fc.cpp
TARGET   := program

ifeq ($(OS),Windows_NT)
    LDLIBS := -lraylib -lopengl32 -lgdi32 -lwinmm
    TARGET := program.exe
    RM     := del /Q
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        LDLIBS := -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
    else
        LDLIBS := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
    endif
    RM := rm -f
endif

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET)

.PHONY: run clean
