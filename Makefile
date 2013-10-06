ifeq ($(strip $(ARCH)),)
	ARCH=x86
endif

CPPFLAGS= \
	-I../ \
	-Iarchs/$(ARCH)

include archs/$(ARCH)/Makefile

OBJS= gaussian_yvv.o \
      convolution.o \
      resample.o \
      subsample.o \
      scale_space.o \
      sift.o \
      canonical_patch.o \
      p3p.o \
      epipolar.o \
      io_pnm.o\
      io_pgm.o\
      timer.o \
      arctan_camera_model.o \
      rational_polynomial_camera_model.o \
      gl/text.o \
      gl/helpers.o

TARGET_OBJS=$(addprefix archs/$(ARCH)/, $(OBJS))

archs/$(ARCH)/%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $^ -c -o $@

TARGET_OBJS+= $(addprefix archs/$(ARCH)/, $(ARCH_OBJS))

LOADLIBES= -lecv -lfltk_gl -lfltk -lGLU -lGL -lstdc++

CFLAGS=-g -Wall -O2 -finline-functions $(ARCH_FLAGS) #-save-temps

CXXFLAGS=$(CFLAGS)

LIBECV=archs/$(ARCH)/libecv.a

$(LIBECV): $(TARGET_OBJS)
	$(AR) rvs $@ $^

depend:
	rm -f .deps
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MM -MG -MP *.cpp > .deps


clean:
	$(RM) $(TARGET_OBJS) $(LIBECV)

-include .deps
