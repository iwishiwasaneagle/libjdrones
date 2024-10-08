cmake_minimum_required(VERSION 3.22)
project(
        ${CMAKE_PROJECT_NAME}Benchmarks
        LANGUAGES CXX
)

verbose_message("Adding benchmarks under ${CMAKE_PROJECT_NAME}Benchmarks...")

find_package(benchmark REQUIRED)

set(bench_sources
        benchmark_envs.cpp
        benchmark_data.cpp
        benchmark_transforms.cpp
        benchmark_polynomial.cpp
)
set(${CMAKE_PROJECT_NAME}_TEST_LIB ${LIB_NAME})

foreach (file ${bench_sources})
    string(REGEX REPLACE "(.*/)([a-zA-Z0-9_ ]+)(\.cpp)" "\\2" bench_name ${file})

    add_executable(${bench_name} ${file})

    target_compile_features(${bench_name} PUBLIC cxx_std_17)

    target_link_libraries(
            ${bench_name}
            PRIVATE
            benchmark::benchmark
            Eigen3::Eigen
            ${${CMAKE_PROJECT_NAME}_TEST_LIB}
    )
endforeach ()


verbose_message("Finished adding benchmarks for ${CMAKE_PROJECT_NAME}.")
