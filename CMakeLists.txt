cmake_minimum_required(VERSION 3.22)
project(
        "jdrones"
        VERSION 0.1.0
        LANGUAGES CXX
)
if (PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there.\n")
endif ()
include(GNUInstallDirs)
include(cmake/SourcesAndHeaders.cmake)
include(cmake/StandardSettings.cmake)
include(cmake/Utils.cmake)
include(cmake/Deps.cmake)
add_clang_format_target()
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fno-math-errno")
set(CMAKE_CXX_STANDARD 23)
message(STATUS "Started CMake for ${PROJECT_NAME} v${PROJECT_VERSION}...\n")

# Set project strings
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWERCASE)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)
set(LIB_NAME ${PROJECT_NAME_LOWERCASE})
set(PY_INSTALL_NAME lib${PROJECT_NAME_LOWERCASE})
set(PY_NAME _core)

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(ignoreMe "${BUILD_PYTHON}")
if((NOT DEFINED BUILD_PYTHON) OR DEFINED DEV)
    # CORE LIBRARY
    add_library(${LIB_NAME} SHARED ${sources} ${headers})
    target_include_directories(
            ${LIB_NAME}
            PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            PRIVATE
            ${PROJECT_SOURCE_DIR}/src
    )
    target_link_libraries(
            ${LIB_NAME}
            PUBLIC
            Eigen3::Eigen
    )
    if (${PROJECT_NAME_UPPERCASE}_ENABLE_VECTORIZATION)
        message(WARNING "Vectorization is in experimental mode and may cause unknown issues")
        include(cmake/EigenVectorization.cmake)
        target_compile_options(
                ${LIB_NAME}
                PRIVATE
                ${_CXX_VECTORIZATION_FLAGS}
        )
    endif ()

    install(
            TARGETS
            ${LIB_NAME}
            EXPORT
            ${LIB_NAME}Targets
            LIBRARY DESTINATION
            ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION
            ${CMAKE_INSTALL_BINDIR}
            ARCHIVE DESTINATION
            ${CMAKE_INSTALL_LIBDIR}
            INCLUDES DESTINATION
            include
            PUBLIC_HEADER DESTINATION
            include
    )

    install(
            EXPORT
            ${LIB_NAME}Targets
            FILE
            ${LIB_NAME}Targets.cmake
            NAMESPACE
            ${LIB_NAME}::
            DESTINATION
            ${CMAKE_INSTALL_LIBDIR}/cmake/${LIB_NAME}
    )
    install(
            DIRECTORY
            include/${LIB_NAME}
            DESTINATION
            include
    )

    set(${LIB_NAME}_CMAKE_PACKAGE_INSTALL_SUBDIR "share/${LIB_NAME}/cmake")
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
            ${LIB_NAME}ConfigVersion.cmake
            VERSION
            ${PROJECT_VERSION}
            COMPATIBILITY
            SameMajorVersion
    )
    configure_package_config_file(
            ${CMAKE_CURRENT_LIST_DIR}/cmake/${LIB_NAME}Config.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}Config.cmake
            INSTALL_DESTINATION
            ${CMAKE_INSTALL_LIBDIR}/cmake/${LIB_NAME}
    )
    install(
            FILES
            ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}Config.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake
            DESTINATION
            ${CMAKE_INSTALL_LIBDIR}/cmake/${LIB_NAME}
    )
endif()
if(DEFINED BUILD_PYTHON OR DEFINED DEV)
    # PYTHON BINDINGS

    # We are using the SKBUILD variable, which is defined within pyproject.toml,
    # running the CMake build, to control building the Python wrapper. This allows
    # the C++ project to be installed, standalone, when using the standard CMake
    # build flow.
    find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
    find_package(pybind11 CONFIG REQUIRED)
    find_package(
            ${LIB_NAME} REQUIRED
            HINTS ${CMAKE_CURRENT_BINARY_DIR}/
    )

    # pybind11.readthedocs.io/en/stable/changelog.html
    if ((Python_VERSION GREATER_EQUAL "3.11") AND (pybind11_VERSION LESS 2.10.1))
        message(FATAL_ERROR "pybind11 version ($pybind11_VERSION) does not support
                             this version of python ($Python_VERSION)"
        )
    endif ()
    if ((Python_VERSION GREATER_EQUAL "3.12") AND (pybind11_VERSION LESS 2.11.0))
        message(FATAL_ERROR "pybind11 version ($pybind11_VERSION) does not support
                             this version of python ($Python_VERSION)"
        )
    endif ()

    python_add_library(${PY_NAME} MODULE ${python_sources} WITH_SOABI)
    target_link_libraries(
            ${PY_NAME}
            PRIVATE
            pybind11::headers
            Eigen3::Eigen
            ${LIB_NAME}::${LIB_NAME}
    )
    install(TARGETS ${PY_NAME} DESTINATION ${PY_INSTALL_NAME})
endif ()

if (${PROJECT_NAME_UPPERCASE}_ENABLE_UNIT_TESTING)
    enable_testing()
    message(STATUS "Build unit tests for the project. Tests should always be found in the test folder\n")
    add_subdirectory(test)
endif ()

if (${PROJECT_NAME_UPPERCASE}_BUILD_BENCHMARKS)
    message(STATUS "Build benchmarks for the project. Benchmarks should always be found in the bench folder\n")
    add_subdirectory(bench)
endif ()

include(cmake/Doxygen.cmake)
