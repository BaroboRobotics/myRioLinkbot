# Set up a toolchain for cross-compilation to the myRIO device. We expect the
# NILRT_SDK environment variable to be set appropriately.

set(CMAKE_SYSTEM_NAME Linux)

# Find the 32-bit xor 64-bit native sysroot.
# TODO test on 32-bit :P
if(    EXISTS $ENV{NILRT_SDK}/sysroots/x86_64-nilrtsdk-linux AND
       NOT EXISTS $ENV{NILRT_SDK}/sysroots/i686-nilrtsdk-linux)
    set(nativeSysroot $ENV{NILRT_SDK}/sysroots/x86_64-nilrtsdk-linux)
elseif(NOT EXISTS $ENV{NILRT_SDK}/sysroots/x86_64-nilrtsdk-linux AND
       EXISTS $ENV{NILRT_SDK}/sysroots/i686-nilrtsdk-linux)
    set(nativeSysroot $ENV{NILRT_SDK}/sysroots/i686-nilrtsdk-linux)
else()
    message(FATAL_ERROR "Unable to find native sysroot (i.e., $ENV{NILRT_SDK}/sysroots/x86_64-nilrtsdk-linux)")
endif()

set(CMAKE_C_COMPILER ${nativeSysroot}/usr/bin/armv7a-vfp-neon-nilrt-linux-gnueabi/arm-nilrt-linux-gnueabi-gcc)
set(CMAKE_CXX_COMPILER ${nativeSysroot}/usr/bin/armv7a-vfp-neon-nilrt-linux-gnueabi/arm-nilrt-linux-gnueabi-g++)

set(CMAKE_SYSROOT "$ENV{NILRT_SDK}/sysroots/armv7a-vfp-neon-nilrt-linux-gnueabi")
