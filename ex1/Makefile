# Plain Makefile alternative to the CMake build. Either one works on
# Linux with gcc 11.4 or above.

CXX      ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -Werror -pedantic -O2
INCLUDES := -Iinclude

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

.PHONY: all clean

all: $(EXEC)

$(EXEC): $(LIB_OBJS) $(MAIN_OBJ)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

clean:
	rm -f $(LIB_OBJS) $(MAIN_OBJ) $(EXEC)
