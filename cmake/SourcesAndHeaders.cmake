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
        include/jdrones/transforms.h
        include/jdrones/constants.h
        include/jdrones/data.h
        include/jdrones/polynomial.h
        include/jdrones/solvers.h
        include/jdrones/controllers.h
        include/jdrones/gymnasium.h

        include/jdrones/dynamics/dynamics.h
        include/jdrones/dynamics/base.h
        include/jdrones/dynamics/linear.h
        include/jdrones/dynamics/nonlinear.h

        include/jdrones/envs/envs.h
        include/jdrones/envs/lqr.h
        include/jdrones/envs/poly/base.h
        include/jdrones/envs/poly/fifth.h
        include/jdrones/envs/poly/opt_fifth.h
)

set(test_sources
        src/test_envs.cpp
        src/test_dynamicdroneenv.cpp
        src/test_data.cpp
        src/test_polynomial.cpp
        src/test_solvers.cpp
        src/test_controller.cpp
)
set(test_headers
        src/utils.h
)