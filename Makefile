CC=g++
CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
LD_FLAGS := 
CC_FLAGS := -Wall -MMD -g 
TARGET=gbemu.out

all: directories ${TARGET}

$(TARGET): $(OBJ_FILES)
	$(CC) $(LD_FLAGS) -o $@ $^

obj/%.o: src/%.cpp
	$(CC) $(CC_FLAGS) -c -o $@ $<

.PHONY: directories clean

directories:
	mkdir -p obj log

clean:
	rm -f $(TARGET) $(OBJ_FILES) $(OBJ_FILES:.o=.d)

# Automatic dependency graph generation with -MMD
-include $(OBJ_FILES:.o=.d)
