CROSS=x86_64-w64-mingw32-
CC=$(CROSS)gcc
CXX=$(CROSS)g++
AR=$(CROSS)ar
LD=$(CROSS)ld


CPPFLAGS+=-I/usr/local/win32/include/
ARCH_FLAGS= -msse -msse2
ARCH_OBJS=arch/yvv.o \
	  arch/sleep.o \
	  arch/system.o


