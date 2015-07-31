cmake_minimum_required(VERSION 2.8.12) 
project(liblinkbot C CXX)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 0)
set(VERSION_PATCH 2)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# Give subprojects access to LinkbotLabs.cmake
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/../cmake-modules")

#include(LinkbotLabs-SDK)
#ll_add_dependency(baromesh/library baromesh)
add_subdirectory(LinkbotLabs-SDK)

if(CMAKE_CROSSCOMPILING)
    find_library(PTHREAD pthread ${CMAKE_PREFIX_PATH} REQUIRED)
    #set(Boost_COMPILER -gcc)
    #add_compile_options(${CMAKE_CROSSCOMPILE_C_FLAGS})
    # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_CROSSCOMPILE_C_FLAGS} -static-libstdc++ -static-libgcc")
endif(CMAKE_CROSSCOMPILING)

##############################################################################
# Boost

# Linking statically to Boost is easier for prototyping. For multi-module
# projects we'll want to use shared libraries and defined -DBOOST_ALL_DYN_LINK
# or the like.
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.54.0 REQUIRED COMPONENTS system log filesystem thread)
find_package(Threads)

set(SOURCES
    src/myriolinkbot.cpp
    src/myriolinkbot.c.cpp
    )

add_library(myrio_linkbot SHARED ${SOURCES})

set_source_files_properties(${SOURCES}
    PROPERTIES COMPILE_FLAGS "-std=c++11")

target_include_directories(myrio_linkbot
    PUBLIC $<TARGET_PROPERTY:sfp,INCLUDE_DIRECTORIES>
           $<TARGET_PROPERTY:robot-interface,INCLUDE_DIRECTORIES>
           $<TARGET_PROPERTY:baromesh,INCLUDE_DIRECTORIES>
           $<TARGET_PROPERTY:cxx-util,INTERFACE_INCLUDE_DIRECTORIES>
           ${Boost_INCLUDE_DIRS}
           include
    )
target_link_libraries(myrio_linkbot
    PUBLIC baromesh
    )

if(WIN32)
    target_link_libraries(myrio_linkbot
        PRIVATE setupapi
        )
endif()

set_target_properties(myrio_linkbot PROPERTIES
    VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
    SOVERSION ${VERSION_MAJOR}
    MACOSX_RPATH ON
    LINK_FLAGS -Wl,-ulinkbotNew
)

add_executable(movewait_test tests/movewait.cpp)
target_link_libraries(movewait_test myrio_linkbot)
target_compile_options(movewait_test PRIVATE "-std=c++11")

add_executable(movewait_c_test tests/movewait.c.cpp)
target_link_libraries(movewait_c_test myrio_linkbot)
target_compile_options(movewait_c_test PRIVATE "-std=c++11")

install(TARGETS myrio_linkbot LIBRARY DESTINATION lib)

set(CPACK_PACKAGE_NAME myrio_liblinkbot)
set(CPACK_PACKAGE_VERSION ${VERSION})
include(CPack)
