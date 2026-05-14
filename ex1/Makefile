# Plain Makefile build. Requires mp-units to be discoverable by the
# compiler -- e.g. installed via the course's vcpkg toolchain, brew, or
# apt. If your mp-units headers live outside the default include path,
# set MP_UNITS_INC to the directory that contains "mp-units/...":
#
#   make MP_UNITS_INC=$VCPKG_ROOT/installed/x64-linux/include
#
# Otherwise prefer the CMake build (which uses find_package(mp-units)).

CXX          ?= g++
CXXFLAGS     ?= -std=c++20 -Wall -Wextra -Werror -pedantic -O2
MP_UNITS_INC ?=
INCLUDES     := -Iinclude $(if $(MP_UNITS_INC),-isystem $(MP_UNITS_INC),)

LIB_SRCS := \
    src/VoxelGrid.cpp \
    src/BuildingMap.cpp \
    src/ConfigParser.cpp \
    src/MapIO.cpp \
    src/LidarMock.cpp \
    src/MovementMock.cpp \
    src/Drone.cpp \
    src/Simulator.cpp

MAIN_SRC := src/main.cpp

LIB_OBJS  := $(LIB_SRCS:.cpp=.o)
MAIN_OBJ  := $(MAIN_SRC:.cpp=.o)

EXEC      := drone_mapper

TEST_SRC  := tests/test_main.cpp
TEST_EXEC := run_tests

.PHONY: all clean test

all: $(EXEC)

$(EXEC): $(LIB_OBJS) $(MAIN_OBJ)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

test: $(TEST_EXEC)
	./$(TEST_EXEC)

$(TEST_EXEC): $(LIB_OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -Itests -o $@ $(TEST_SRC) $(LIB_OBJS)

clean:
	rm -f $(LIB_OBJS) $(MAIN_OBJ) $(EXEC) $(TEST_EXEC)
