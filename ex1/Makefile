# Plain Makefile alternative to the CMake build. Either one works on
# Linux with gcc 11.4 or above.

CXX      ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -Werror -pedantic -O2
MP_UNITS  := third_party/mp-units/src
GSL_LITE  := third_party/gsl-lite/include
INCLUDES  := -Iinclude \
             -I$(MP_UNITS)/core/include \
             -I$(MP_UNITS)/systems/si/include \
             -I$(MP_UNITS)/systems/isq/include \
             -I$(GSL_LITE) \
             -Dgsl_FEATURE_GSL_COMPATIBILITY_MODE=1

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
