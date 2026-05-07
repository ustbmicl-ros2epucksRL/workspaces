#---------------------------------------------------------------------
# Makefile - Khepera Library


# Revision 2.1  2016/06/17 18:51:54  jtharin


#---------------------------------------------------------------------

# Please read README.kteam for more information about libkhepera 
# installation procedure.

# The shell to use for scripts
SHELL		= /bin/sh


KTEAM_KERNEL_VERSION = 3.18.18-custom

# Specify the target system. The corresponding sytem specify
# section bellow should be edited as well.
TARGET_SYSTEM	= khepera-${KTEAM_KERNEL_VERSION}


# libkhepera configuration files directory
KB_CONFIG_DIRECTORY=/etc/libkhepera

KB_FLAGS = -DKB_CONFIG_DIRECTORY=\"$(KB_CONFIG_DIRECTORY)\"


# Modify this!
KTEAM_HOME = `pwd`/..

# Modify this maybe these
LIBKHEPERA_ROOT  = `pwd`

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
export CFLAGS= -O2 -pipe -g -feliminate-unused-debug-types
export CXXFLAGS= -O2 -pipe -g -feliminate-unused-debug-types
export LDFLAGS=-Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed
export CPPFLAGS=
export KCFLAGS=--sysroot=${SDKTARGETSYSROOT}
export OECORE_DISTRO_VERSION=1.8
export OECORE_SDK_VERSION=1.8
export ARCH=arm
export CROSS_COMPILE=arm-poky-linux-gnueabi-
#### end of POKY exports


#OE_HOME = /usr/local/khepera4-yocto-2.0

#KHEPERA_TOOLS = ${OE_HOME}



#KTEAM_KERNEL_HOME = ${KHEPERA_TOOLS}/build/tmp/sysroots/overo/usr/src

# And don't touch these
#ARCH = armv7-a
#CROSS_COMPILE = arm-poky-linux-gnueabi-
#PATH := $(PATH):${KHEPERA_TOOLS}/build/tmp/sysroots/i686-linux/usr/bin/arm-poky-linux-gnueabi

#CC = ${CROSS_COMPILE}gcc  -march=armv7-a -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=${KHEPERA_TOOLS}/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi
#CXX = ${CROSS_COMPILE}g++  -march=armv7-a -mthumb-interwork -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=${KHEPERA_TOOLS}/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi
#LD = ${CROSS_COMPILE}ld
#AR = ${CROSS_COMPILE}ar
#AS = ${CROSS_COMPILE}as


#INCPATH = ${LIBKHEPERA_ROOT}/build-${TARGET_SYSTEM}/include


#LIBPATH = ${LIBKHEPERA_ROOT}/build-${TARGET_SYSTEM}/lib


# Pointer to the libkhepera build directory
#LIBKHEPERA = ${LIBKHEPERA_ROOT}/build-${TARGET_SYSTEM}



#---------------------------------------------------------------------

# see also khepera.h for version:  KB_VERSION KB_REVISION
LIBVER		= 2.1
LIBNAME		= libkhepera

SRCS		= $(wildcard *.c)
OBJS		= $(patsubst %.c,%.o,${SRCS})

.PHONY: all clean depend doc docs tests clean_tests clean_doc

# Exports to submakes
export SHELL #CC AR LD SYS_INCLUDES AS
export LIBNAME LIBVER TARGET_SYSTEM
export KB_FLAGS

#
# Rules
#
all: 	
	@cd src && make -w 

doc: docs

docs:
	@cd src && make -w docs

lib:
	@cd src && make -w
	
tests:
	@cd src/tests && make

clean:
	@rm -rf build-${TARGET_SYSTEM}
	@cd src && make -w clean
	
clean_tests: 
	@cd src/tests && make -w clean	

clean_doc:
	@rm doc/html/*  

depend:	
	@cd src && make -w depend

