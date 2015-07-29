# Set up initial C and C++ flags for the nilrt cross-compiler. The values in
# this file came from the environment-setup-armv7a-vfp-neon-nilrt-linux-gnueabi
# script in the nilrt SDK.

set(CMAKE_C_FLAGS_INIT "-march=armv7-a -mthumb-interwork -mfloat-abi=softfp -mfpu=neon -fPIC")
set(CMAKE_C_FLAGS_DEBUG_INIT "-g")
set(CMAKE_C_FLAGS_MINSIZEREL_INIT "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE_INIT "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")

set(CMAKE_CXX_FLAGS_INIT "-march=armv7-a -mthumb-interwork -mfloat-abi=softfp -mfpu=neon -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-g")
set(CMAKE_CXX_FLAGS_MINSIZEREL_INIT "-Os -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")

set(ldflags "-Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed -static-libgcc -static-libstdc++")

set(CMAKE_EXE_LINKER_FLAGS_INIT ${ldflags})
set(CMAKE_EXE_LINKER_FLAGS_DEBUG_INIT ${ldflags})
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL_INIT ${ldflags})
set(CMAKE_EXE_LINKER_FLAGS_RELEASE_INIT ${ldflags})
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO ${ldflags})

set(CMAKE_SHARED_LINKER_FLAGS_INIT ${ldflags})
set(CMAKE_SHARED_LINKER_FLAGS_DEBUG_INIT ${ldflags})
set(CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL_INIT ${ldflags})
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE_INIT ${ldflags})
set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO ${ldflags})

set(CMAKE_MODULE_LINKER_FLAGS_INIT ${ldflags})
set(CMAKE_MODULE_LINKER_FLAGS_DEBUG_INIT ${ldflags})
set(CMAKE_MODULE_LINKER_FLAGS_MINSIZEREL_INIT ${ldflags})
set(CMAKE_MODULE_LINKER_FLAGS_RELEASE_INIT ${ldflags})
set(CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO ${ldflags})
