cmake_minimum_required(VERSION 3.22)
project(
        ${CMAKE_PROJECT_NAME}Tests
        LANGUAGES CXX
)

verbose_message("Adding tests under ${CMAKE_PROJECT_NAME}Tests...")

find_package(Catch2 3 CONFIG REQUIRED)
include(../cmake/Deps.cmake)
include(CTest)
include(Catch)

if (${PROJECT_NAME_UPPERCASE}_ENABLE_CODE_COVERAGE)
    target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC -O0 -g -fprofile-arcs -ftest-coverage)
    target_link_options(${CMAKE_PROJECT_NAME} PUBLIC -fprofile-arcs -ftest-coverage)
    verbose_message("Code coverage is enabled and provided with GCC.")
endif ()

set(${CMAKE_PROJECT_NAME}_TEST_LIB ${LIB_NAME})

foreach (file ${test_sources})
    string(REGEX REPLACE "(.*/)([a-zA-Z0-9_ ]+)(\.cpp)" "\\2" test_name ${file})

    add_executable(${test_name}_Tests ${file} ${test_headers})

    target_compile_features(${test_name}_Tests PUBLIC cxx_std_17)

    target_link_libraries(
            ${test_name}_Tests
            PRIVATE
            Catch2::Catch2WithMain
            Eigen3::Eigen
            ${${CMAKE_PROJECT_NAME}_TEST_LIB}
    )
    catch_discover_tests(${test_name}_Tests)
endforeach ()

if (1)

    add_executable(all_Tests ${test_sources})

    target_link_libraries(
            all_Tests
            PRIVATE
            Catch2::Catch2WithMain
            Eigen3::Eigen
            ${${CMAKE_PROJECT_NAME}_TEST_LIB}
    )

    catch_discover_tests(all_Tests)
endif ()

verbose_message("Finished adding unit tests for ${CMAKE_PROJECT_NAME}.")
