set(sources
        src/envs.cpp
        src/dynamics.cpp
        src/transforms.cpp
        src/polynomial.cpp
        src/solvers.cpp
        src/controllers.cpp
)

set(python_sources
        src/python.cpp
)

set(headers
        include/jdrones/dynamics.h
        include/jdrones/envs.h
        include/jdrones/transforms.h
        include/jdrones/constants.h
        include/jdrones/data.h
        include/jdrones/polynomial.h
        include/jdrones/solvers.h
        include/jdrones/types.h
        include/jdrones/controllers.h
)

set(test_sources
        src/test_dynamicdroneenv.cpp
        src/test_data.cpp
        src/test_polynomial.cpp
        src/test_solvers.cpp
)
