MAKEFLAGS += -j3 # parallel processes
CXX				= g++
CXX_FLAGS = -std=c++11 -Wall -O3 -march=native -funroll-loops -fstrict-aliasing
INCLUDES 	= -I/usr/include/drm -I/usr/include/libdrm -Isrc/sketches -Isrc
LIBS     	= -lEGL -lGLESv2 -lgbm -ldrm -lpthread -lm

SDL2_CFLAGS := $(shell pkg-config --cflags sdl2 2>/dev/null)
SDL2_LIBS := $(shell pkg-config --libs sdl2 2>/dev/null)
RASPI_BUILD ?= $(shell [ -r /sys/firmware/devicetree/base/model ] && grep -aq "Raspberry Pi" /sys/firmware/devicetree/base/model && echo 1 || echo 0)
ifneq ($(RASPI_BUILD),1)
ifneq ($(strip $(SDL2_LIBS)),)
CXX_FLAGS += -DHAS_SDL2=1
INCLUDES  += $(SDL2_CFLAGS)
LIBS      += $(SDL2_LIBS)
else
CXX_FLAGS += -DHAS_SDL2=0
endif
else
CXX_FLAGS += -DHAS_SDL2=0
endif

BIN 			= igr
SRC_DIR 	= ./src
BIN_DIR 	= ./bin
OUT_DIR 	= ./out

# Source files
CPP = $(wildcard $(SRC_DIR)/*.cpp) \
			$(wildcard $(SRC_DIR)/lib/*.cpp) \
			$(wildcard $(SRC_DIR)/sketches/*.cpp) \
			$(wildcard $(SRC_DIR)/sketches/**/*.cpp)

# All .o files go to build dir
OBJ = $(patsubst $(SRC_DIR)/%.cpp, $(OUT_DIR)/%.o, $(CPP))

# GCC will create these .d files containing dependencies
DEP = $(OBJ:%.o=%.d)

# Default target
$(BIN) : $(BIN_DIR)/$(BIN)

# Link binary
$(BIN_DIR)/$(BIN) : $(OBJ)
	mkdir -p $(@D)
	$(CXX) $(CXX_FLAGS) $^ -o $@ $(LIBS)

# Include all .d files
-include $(DEP)

# Compile source to object (and generate dependency file)
$(OUT_DIR)/%.o: $(SRC_DIR)/%.cpp
	mkdir -p $(@D)
	$(CXX) $(CXX_FLAGS) $(INCLUDES) -MMD -c $< -o $@

# Build shader.h files with embedded shaders
shaders: 
	$(MAKE) -C src/sketches

.PHONY : clean
clean :
	rm -rf -- $(OUT_DIR) $(BIN_DIR)/$(BIN)

