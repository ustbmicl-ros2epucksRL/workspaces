#
# template Makefile for development with libkhepera
#  waning: path dependant; see KTEAM_HOME and LIBKHEPERA VARIABLES
#

# Modify this!
KTEAM_HOME := $(shell pwd)/..

# And maybe these


export TOOL_DIR=/opt/poky/1.8


#### POKY exports
export SDKTARGETSYSROOT=${TOOL_DIR}/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi
export PATH:=${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/bin:${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi:${PATH}

export CCACHE_PATH:=${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/bin:${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi:${CCACHE_PATH}
export PKG_CONFIG_SYSROOT_DIR=${SDKTARGETSYSROOT}
export PKG_CONFIG_PATH=${SDKTARGETSYSROOT}/usr/lib/pkgconfig
export CONFIG_SITE=${TOOL_DIR}/site-config-cortexa8hf-vfp-neon-poky-linux-gnueabi
export OECORE_NATIVE_SYSROOT=${TOOL_DIR}/sysroots/i686-pokysdk-linux
export OECORE_TARGET_SYSROOT=${SDKTARGETSYSROOT}
export OECORE_ACLOCAL_OPTS=-I ${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr/share/aclocal
export PYTHONHOME=${TOOL_DIR}/sysroots/i686-pokysdk-linux/usr
export CC=arm-poky-linux-gnueabi-gcc  -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=${SDKTARGETSYSROOT}
export CXX=arm-poky-linux-gnueabi-g++  -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=${SDKTARGETSYSROOT}
export CPP=arm-poky-linux-gnueabi-gcc -E  -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=${SDKTARGETSYSROOT}
export AS=arm-poky-linux-gnueabi-as
export LD=arm-poky-linux-gnueabi-ld  --sysroot=${SDKTARGETSYSROOT}
export GDB=arm-poky-linux-gnueabi-gdb
export STRIP=arm-poky-linux-gnueabi-strip
export RANLIB=arm-poky-linux-gnueabi-ranlib
export OBJCOPY=arm-poky-linux-gnueabi-objcopy
export OBJDUMP=arm-poky-linux-gnueabi-objdump
export AR=arm-poky-linux-gnueabi-ar
export NM=arm-poky-linux-gnueabi-nm
export M4=m4
export TARGET_PREFIX=arm-poky-linux-gnueabi-
export CONFIGURE_FLAGS=--target=arm-poky-linux-gnueabi --host=arm-poky-linux-gnueabi --build=i686-linux --with-libtool-sysroot=${SDKTARGETSYSROOT}
export CFLAGS= -O2 -pipe -g -feliminate-unused-debug-types -std=gnu99 -fPIC -Wall -Wno-unused-parameter -Wno-unused-function -fno-strict-overflow
export CXXFLAGS= -O2 -pipe -g -feliminate-unused-debug-types
export LDFLAGS=-Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed
export CPPFLAGS=
export KCFLAGS=--sysroot=${SDKTARGETSYSROOT}
export OECORE_DISTRO_VERSION=1.8
export OECORE_SDK_VERSION=1.8
export ARCH=arm
export CROSS_COMPILE=arm-poky-linux-gnueabi-
#### end of POKY exports

LIBKHEPERA_ROOT = ${KTEAM_HOME}

KTEAM_KERNEL_VERSION = 3.18.18-custom

TARGET_SYSTEM	= khepera-${KTEAM_KERNEL_VERSION}

INCPATH = ${KTEAM_HOME}/build-${KTEAM_KERNEL_VERSION}/include
#INCPATH = ${OE_HOME}/build/tmp/sysroots/overo/usr/include

LIBPATH = ${KTEAM_HOME}/build-${KTEAM_KERNEL_VERSION}/lib
#LIBPATH = ${OE_HOME}/build/tmp/sysroots/overo/usr/lib

# Pointer to the libkhepera build directory
LIBKHEPERA = ${LIBKHEPERA_ROOT}/build-${TARGET_SYSTEM}


SRCS	= $(wildcard *.c)

# Add nanopb sources
NANO_SRCS = nanopb/pb_encode.c  # The nanopb encoder
NANO_SRCS += nanopb/pb_decode.c  # The nanopb decoder
NANO_SRCS += nanopb/pb_common.c  # The nanopb common parts
NANO_SRCS += ../robosar_messages/proto/include/c/robosar.pb.c # robosar library file

NANO_INC = -I nanopb
NANO_INC += -I ../robosar_messages/proto/include/c

OBJS	= $(patsubst %.c,%.o,${SRCS})
INCS	= -I ${LIBKHEPERA}/include -I. -I apriltag
LIBS	= -L ${LIBKHEPERA}/lib  -L. -lkhepera -lpthread -lm -lapriltag

#CFLAGS 	= 

APRILTAG_SRCS := apriltag/apriltag.c apriltag/apriltag_pose.c apriltag/apriltag_quad_thresh.c apriltag/common/g2d.c apriltag/common/getopt.c apriltag/common/homography.c apriltag/common/image_u8.c apriltag/common/image_u8x3.c apriltag/common/image_u8x4.c apriltag/common/matd.c apriltag/common/pam.c apriltag/common/pjpeg.c apriltag/common/pjpeg-idct.c apriltag/common/pnm.c apriltag/common/string_util.c apriltag/common/svd22.c apriltag/common/time_util.c apriltag/common/unionfind.c apriltag/common/workerpool.c apriltag/common/zarray.c apriltag/common/zhash.c apriltag/common/zmaxheap.c apriltag/tag16h5.c apriltag/tag25h9.c apriltag/tag36h11.c apriltag/tagCircle21h7.c apriltag/tagCircle49h12.c apriltag/tagCustom48h12.c apriltag/tagStandard41h12.c apriltag/tagStandard52h13.c
APRILTAG_HEADERS := $(shell ls apriltag/*.h apriltag/common/*.h)
APRILTAG_OBJS := $(APRILTAG_SRCS:%.c=%.o)

# for debugging
#CFLAGS 	= -g

TARGET	= template_perception template-static 

.PHONY: all clean depend

template_perception: prog-template.o
	@echo "Building $@"
	$(CC) $(NANO_INC) -o $@ $? ${NANO_SRCS} $(LIBS) $(CFLAGS) 

libapriltag.a: $(APRILTAG_OBJS)
	@echo "  [$@]"
	@$(AR) -cq $@ $(APRILTAG_OBJS)

template-static: prog-template.o
	@echo "Building $@"
	$(CC) -o $@ $? $(LIBS) -static $(CFLAGS)

all: 	${TARGET}

clean : 
	@echo "Cleaning"
	@rm -f *.o .depend ${TARGET} *~

depend:	
	@echo "Building dependencies"
	@rm -f .depend
	@touch .depend
	@makedepend ${SYS_INCLUDES} ${INCS} -Y -f .depend ${SRCS}

%.o:	%.c
	@echo "Compiling $@"
	@$(CC) $(INCS) $(NANO_INC) -c $(CFLAGS) $< -o $@

libs: libapriltag.a

ifeq (.depend,$(wildcard .depend))
include .depend 
endif
