CPPFLAGS= -I../../../ -I ./

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
