cmake_minimum_required(VERSION 3.22)

project(va_synthesis VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(VA_BUILD_APPS "Build command-line applications" ON)
option(VA_BUILD_TESTS "Build tests" ON)
option(VA_ENABLE_BRT "Expose the vendored BRTLibrary headers to the geometrical backend" ON)
option(VA_ENABLE_PFFDTD "Enable the external PFFDTD backend adapter" ON)
option(VA_ENABLE_PFFDTD_E2E_TESTS
    "Run the opt-in headless PFFDTD Python integration test" OFF)
set(VA_PFFDTD_PYTHON_EXECUTABLE "" CACHE FILEPATH
    "Python interpreter in the PFFDTD environment")

add_library(va_synthesis
    src/core/audio.cpp
    src/core/engine.cpp
    src/geometrical/brt_geometrical_solver.cpp
    src/hybrid/hybrid_solver.cpp
    src/wave/fdtd_solver.cpp
    src/wave/pffdtd_backend.cpp
    src/wave/pffdtd_scene.cpp
)
add_library(va::synthesis ALIAS va_synthesis)
set_target_properties(va_synthesis PROPERTIES EXPORT_NAME synthesis)

target_compile_features(va_synthesis PUBLIC cxx_std_20)
target_include_directories(va_synthesis
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(MSVC)
    target_compile_options(va_synthesis PRIVATE /W4 /permissive-)
else()
    target_compile_options(va_synthesis PRIVATE -Wall -Wextra -Wpedantic)
endif()

set(VA_PFFDTD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/submodules/pffdtd")
if(VA_ENABLE_PFFDTD)
    if(NOT EXISTS "${VA_PFFDTD_DIR}/python/fdtd/sim_fdtd.py")
        message(FATAL_ERROR
            "PFFDTD is not initialized. Run: git submodule update --init --recursive")
    endif()
    target_compile_definitions(va_synthesis PUBLIC VA_HAS_PFFDTD=1)
else()
    target_compile_definitions(va_synthesis PUBLIC VA_HAS_PFFDTD=0)
endif()

set(VA_BRT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/submodules/BRTLibrary/include")
set(VA_BRT_EIGEN_INCLUDE_DIR
    "${VA_BRT_INCLUDE_DIR}/third_party_libraries/eigen")
set(VA_BRT_BOOST_INCLUDE_DIR
    "${VA_BRT_INCLUDE_DIR}/third_party_libraries/boost_circular_buffer")
if(VA_ENABLE_BRT)
    if(NOT EXISTS "${VA_BRT_INCLUDE_DIR}/BRTLibrary.h")
        message(FATAL_ERROR
            "BRTLibrary is not initialized. Run: git submodule update --init --recursive")
    endif()
    if(NOT EXISTS "${VA_BRT_EIGEN_INCLUDE_DIR}/Eigen/Core")
        message(FATAL_ERROR
            "BRTLibrary nested dependencies are not initialized. Run: "
            "git submodule update --init --recursive")
    endif()
    set(VA_BRT_COMPAT_DIR "${CMAKE_CURRENT_BINARY_DIR}/brt_compat")
    find_program(VA_PYTHON3 NAMES python3 python REQUIRED)
    execute_process(
        COMMAND "${VA_PYTHON3}"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/patch_brt_wall.py"
            "${VA_BRT_INCLUDE_DIR}/ServiceModules/Wall.hpp"
            "${VA_BRT_COMPAT_DIR}/ServiceModules/Wall.hpp"
        RESULT_VARIABLE VA_BRT_WALL_PATCH
    )
    if(NOT VA_BRT_WALL_PATCH EQUAL 0)
        message(FATAL_ERROR "Failed to create the BRT Wall.hpp compatibility header")
    endif()
    # BRT's CWall::GetNormal returns a reference to a local.  Shadow the header
    # so image-source visibility works in optimized builds without editing BRT.
    target_include_directories(va_synthesis SYSTEM PRIVATE
        "${VA_BRT_COMPAT_DIR}"
        "${VA_BRT_INCLUDE_DIR}"
        "${VA_BRT_EIGEN_INCLUDE_DIR}"
        "${VA_BRT_BOOST_INCLUDE_DIR}"
    )
    target_compile_definitions(va_synthesis PUBLIC VA_HAS_BRT=1)
else()
    target_compile_definitions(va_synthesis PUBLIC VA_HAS_BRT=0)
endif()

install(TARGETS va_synthesis
    EXPORT va-synthesis-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PATTERN ".DS_Store" EXCLUDE)
install(PROGRAMS
    tools/pffdtd_bridge.py
    tools/prepare_pffdtd_job.py
    DESTINATION ${CMAKE_INSTALL_DATADIR}/va-synthesis/tools
)

configure_package_config_file(
    cmake/va-synthesis-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/va-synthesis-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/va-synthesis
    PATH_VARS CMAKE_INSTALL_DATADIR
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/va-synthesis-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(EXPORT va-synthesis-targets
    FILE va-synthesis-targets.cmake
    NAMESPACE va::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/va-synthesis
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/va-synthesis-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/va-synthesis-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/va-synthesis
)

if(VA_BUILD_APPS)
    add_executable(va_demo apps/va_demo.cpp)
    target_link_libraries(va_demo PRIVATE va::synthesis)
endif()

include(CTest)
if(BUILD_TESTING AND VA_BUILD_TESTS)
    add_executable(va_tests tests/test_main.cpp)
    target_link_libraries(va_tests PRIVATE va::synthesis)
    add_test(NAME va_tests COMMAND va_tests)

    if(VA_ENABLE_PFFDTD_E2E_TESTS)
        if(NOT VA_ENABLE_PFFDTD)
            message(FATAL_ERROR "VA_ENABLE_PFFDTD_E2E_TESTS requires VA_ENABLE_PFFDTD")
        endif()
        if(NOT EXISTS "${VA_PFFDTD_PYTHON_EXECUTABLE}")
            message(FATAL_ERROR
                "Set VA_PFFDTD_PYTHON_EXECUTABLE to the PFFDTD environment's Python")
        endif()
        add_executable(va_pffdtd_e2e_fixture tests/pffdtd_e2e_fixture.cpp)
        target_link_libraries(va_pffdtd_e2e_fixture PRIVATE va::synthesis)
        add_test(NAME va_pffdtd_e2e
            COMMAND "${VA_PFFDTD_PYTHON_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_pffdtd_e2e.py"
                --fixture $<TARGET_FILE:va_pffdtd_e2e_fixture>
                --repository "${VA_PFFDTD_DIR}"
                --prepare "${CMAKE_CURRENT_SOURCE_DIR}/tools/prepare_pffdtd_job.py"
                --bridge "${CMAKE_CURRENT_SOURCE_DIR}/tools/pffdtd_bridge.py"
                --python "${VA_PFFDTD_PYTHON_EXECUTABLE}"
        )
        set_tests_properties(va_pffdtd_e2e PROPERTIES TIMEOUT 300)
    endif()
endif()
