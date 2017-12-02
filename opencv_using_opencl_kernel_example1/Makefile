# OpenCV + OpenCL Makefile template
#include ./config.mak
#get all c++ files.
SRCS=$(wildcard *.cpp)
OBJS=$(addprefix objs/, $(patsubst %.cpp,%.o,$(SRCS)))
DEPS=$(addprefix objs/, $(patsubst %.cpp,%.d,$(SRCS)))

CXX=g++
DEBUG = -g
CFLAGS= -Wall `pkg-config --cflags opencv` #-std=c++11
LIBS=`pkg-config --libs opencv`

# OpenCL Dynamic Linking
#PREFIX=/usr/local/cuda/targets/x86_64-linux
LFLAGS=-lOpenCL #-L$(PREFIX)/lib -I$(PREFIX)/include 

LOCAL_CFLAGS=-Wno-deprecated-declarations

TARGET=main #lane_detection_and_tracking

.PHONY: all clean execute directories

all	:	directories $(TARGET)

$(TARGET)	:	$(OBJS)
	@echo "  LINK  $@"
	@$(CXX) $^ -o $@ $(DEBUG) $(CFLAGS) $(LIBS) $(LFLAGS) $(LOCAL_CFLAGS)

objs/%.o	:	%.cpp
	@echo "  CXX    $@"
	@$(CXX) -c $< -o $@ $(CFLAGS) $(LOCAL_CFLAGS)

objs/%.d	:	%.cpp
	@$(CXX) -MM $^ -MT $(patsubst %.d,%.o,$@) > $@

directories	:	
	@mkdir -p objs

clean	:	
	rm -f objs/* $(TARGET)

execute	:	all
	./$(TARGET)

# Include automatically generated dependency files
-include $(DEPS)