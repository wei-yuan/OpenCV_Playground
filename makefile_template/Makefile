#include ./config.mak -> you can move all the FLAG to here and make file without config.mak file
#get all c++ files.
SRCS=$(wildcard *.cpp)
OBJS=$(addprefix objs/, $(patsubst %.cpp,%.o,$(SRCS)))
DEPS=$(addprefix objs/, $(patsubst %.cpp,%.d,$(SRCS)))

CXX=g++

CFLAGS=-g -Wall `pkg-config --cflags opencv` 
LIBS = `pkg-config --libs opencv`

PREFIX=/usr/local/cuda/targets/x86_64-linux
LFLAGS= -L$(PREFIX)/lib -I$(PREFIX)/include -lOpenCL

LOCAL_CFLAGS=-Wno-deprecated-declarations

TARGET=example 

.PHONY: all clean execute directories

all	:	directories $(TARGET)

$(TARGET)	:	$(OBJS)
	@echo "  LINK  $@"
	@$(CXX) $^ -o $@ $(CFLAGS) $(LIBS) $(LFLAGS) $(LOCAL_CFLAGS)

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