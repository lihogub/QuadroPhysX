cmake_minimum_required(VERSION 3.16)
project(QuadroPhysX)

set(CMAKE_CXX_STANDARD 14)
add_compile_definitions(NDEBUG)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_definitions(RELEASE)
endif()

include_directories(include)
file(GLOB SRC_FILES src/*.cpp src/task/*.cpp)

add_executable(QuadroPhysX main.cpp ${SRC_FILES})

#### Build PhysX library ####
# PHYSX_PATH - path to the `{cloned repository}/physx` repo directory git://github.com/NVIDIAGameWorks/PhysX.git
set( PHYSX_ROOT_DIR $ENV{PHYSX_PATH} )
set( PHYSX_INCLUDE_DIRS $ENV{PHYSX_PATH}/include/ $ENV{PHYSX_PATH}/../pxshared/include/ )
set( PHYSX_LIBRARIES
        PhysXExtensions
        PhysX
        PhysXPvdSDK
        PhysXVehicle
        PhysXCharacterKinematic
        PhysXCooking
        PhysXCommon
        PhysXFoundation
        )

set(TARGET_BUILD_PLATFORM "windows")
set(PX_BUILDSNIPPETS OFF CACHE BOOL "Generate the snippets")
set(PX_BUILDPUBLICSAMPLES OFF CACHE BOOL "Generate the samples projects")
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "Generate static libraries")
set(PX_FLOAT_POINT_PRECISE_MATH OFF CACHE BOOL "Float point precise math")
set(NV_USE_STATIC_WINCRT ON CACHE BOOL "Use the statically linked windows CRT")
set(NV_USE_DEBUG_WINCRT ON CACHE BOOL "Use the debug version of the CRT")
set(PXSHARED_PATH $ENV{PHYSX_PATH}/../pxshared)
set(PXSHARED_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CMAKEMODULES_VERSION "1.27")
set(CMAKEMODULES_PATH $ENV{PHYSX_PATH}/../externals/cmakemodules)
set(PX_OUTPUT_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/externals/physx)
set(PX_OUTPUT_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/externals/physx)

# Call into PhysX's CMake scripts
add_subdirectory($ENV{PHYSX_PATH}/compiler/public externals/physx)

# Add physx libraries to target
target_link_libraries(QuadroPhysX PUBLIC ${PHYSX_LIBRARIES})

#if (TARGET_BUILD_PLATFORM STREQUAL "windows")
#    # References NvidiaBuildOptions.cmake to figure out if system is 32/64 bit
#    IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
#        SET(LIBPATH_SUFFIX "64")
#    ELSE()
#        SET(LIBPATH_SUFFIX "32")
#    ENDIF()
#    GetPlatformBinName(PLATFORM_BIN_NAME ${LIBPATH_SUFFIX})
#    set(PhysxOutputPath ${PX_OUTPUT_LIB_DIR}/bin/${PLATFORM_BIN_NAME}/)
#    message("Physx Output Path: " ${PhysxOutputPath})
#
#    # copy PhysX dll's to build dir. Happens on every build.
#    add_custom_command(TARGET QuadroPhysX POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${PhysxOutputPath}" "$<TARGET_FILE_DIR:QuadroPhysX>/cmake-build-profile-visual-studio")
#endif()