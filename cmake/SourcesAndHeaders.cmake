set(sources
        src/envs.cpp
        src/transforms.cpp
        src/polynomial.cpp
        src/solvers.cpp
)

set(python_sources
        src/python.cpp
)

set(headers
        include/jdrones/envs.h
        include/jdrones/transforms.h
        include/jdrones/constants.h
        include/jdrones/data.h
        include/jdrones/polynomial.h
        include/jdrones/solvers.h
        include/jdrones/types.h
)

set(test_sources
        src/test_dynamicdroneenv.cpp
        src/test_data.cpp
        src/test_polynomial.cpp
        src/test_solvers.cpp
)
