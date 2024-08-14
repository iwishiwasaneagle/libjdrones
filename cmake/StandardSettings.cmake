#
# Project settings
#

option(${PROJECT_NAME_UPPERCASE}_BUILD_EXECUTABLE "Build the project as an executable, rather than a library." OFF)
option(${PROJECT_NAME_UPPERCASE}_BUILD_PYTHON_BINDINGS "Build the python bindings using pybind11." ON)
option(${PROJECT_NAME_UPPERCASE}_WARNINGS_AS_ERRORS "Treat compiler warnings as errors." OFF)
option(${PROJECT_NAME_UPPERCASE}_ENABLE_UNIT_TESTING "Enable unit tests for the projects (from the `test` subfolder)." OFF)
option(${PROJECT_NAME_UPPERCASE}_ENABLE_VECTORIZATION "Enable Eigen3 vectorization." OFF)
option(${PROJECT_NAME_UPPERCASE}_BUILD_BENCHMARKS "Build benchmarks." OFF)


#
# Static analyzers
#
# Currently supporting: Clang-Tidy, Cppcheck.

option(${PROJECT_NAME_UPPERCASE}_ENABLE_CLANG_TIDY "Enable static analysis with Clang-Tidy." OFF)
option(${PROJECT_NAME_UPPERCASE}_ENABLE_CPPCHECK "Enable static analysis with Cppcheck." OFF)

#
# Code coverage
#

option(${PROJECT_NAME_UPPERCASE}_ENABLE_CODE_COVERAGE "Enable code coverage through GCC." OFF)

#
# Miscelanious options
#

# Generate compile_commands.json for clang based tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(${PROJECT_NAME_UPPERCASE}_VERBOSE_OUTPUT "Enable verbose output, allowing for a better understanding of each step taken." OFF)

option(${PROJECT_NAME_UPPERCASE}_ENABLE_CCACHE "Enable the usage of Ccache, in order to speed up rebuild times." ON)
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()
