CC=gcc
CXX=g++

CPPFLAGS= \
	-I../

LDFLAGS= -pg -L./

OBJS= gaussian_yvv.o \
      scale_space.o \
      sift.o \
      canonical_patch.o \
      p3p.o \
      epipolar.o \
      timer.o \
      arctan_camera_model.o \
      gl/window_fltk.o \
      gl/helpers.o

LOADLIBES= -lecv -lfltk_gl -lfltk -lGLU -lGL -lstdc++

CFLAGS=-g -Wall -O2 -finline-functions -pg #-save-temps

CXXFLAGS=$(CFLAGS)

libecv.a: $(OBJS)
	$(AR) rvs $@ $^

depend:
	rm -f .deps
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MM -MG -MP *.cpp > .deps


clean:
	rm $(OBJS)
	rm libecv.a

-include .deps
