# cmake_minimum_required(VERSION 3.16)
# project(acado_test)

# find_package(ACADO REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(PythonLibs REQUIRED)

# include_directories(${Eigen_INCLUDE_DIRS})
# include_directories(${PYTHON_INCLUDE_DIRS})
# include_directories(${CMAKE_SOURCE_DIR}/include)
# ####################################################
# add_executable(ctrl_mpc_code_gen)
# target_sources(ctrl_mpc_code_gen PUBLIC
#     ctrl_mpc_code_gen.cpp
# )
# target_include_directories(ctrl_mpc_code_gen PUBLIC
#     ${ACADO_INCLUDE_DIRS}
# )
# target_link_directories(ctrl_mpc_code_gen PUBLIC
#     ${ACADO_LIBRARY_DIRS}
# )
# target_link_libraries(ctrl_mpc_code_gen PUBLIC
#     ${ACADO_SHARED_LIBRARIES}
# )

# ####################################################
# add_executable(ctrl_mpc_sim)
# target_sources(ctrl_mpc_sim PUBLIC
#     ctrl_mpc_sim.cpp
# )
# target_include_directories(ctrl_mpc_sim PUBLIC
#     ${ACADO_INCLUDE_DIRS}
# )
# target_link_directories(ctrl_mpc_sim PUBLIC
#     ${ACADO_LIBRARY_DIRS}
# )
# target_link_libraries(ctrl_mpc_sim PUBLIC
#     ${ACADO_SHARED_LIBRARIES}
# )
# target_link_libraries(ctrl_mpc_sim PUBLIC 
#     ${PYTHON_LIBRARIES}
# )
































cmake_minimum_required(VERSION 3.10)
project(ctrl_mpc_sim)


find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(include)
include_directories(include/prob2_mpc_export)
include_directories(include/qpoases)
include_directories(include/qpoases/SRC)
include_directories(include/qpoases/INCLUDE)

add_library(mpc_solver
include/qpoases/SRC/Bounds.cpp
include/qpoases/SRC/Constraints.cpp
include/qpoases/SRC/CyclingManager.cpp
include/qpoases/SRC/Indexlist.cpp
include/qpoases/SRC/MessageHandling.cpp
include/qpoases/SRC/MessageHandling.ipp
include/qpoases/SRC/QProblemB.cpp
include/qpoases/SRC/QProblem.cpp
include/qpoases/SRC/SubjectTo.cpp
include/qpoases/SRC/Utils.cpp
include/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
include/prob2_mpc_export/acado_auxiliary_functions.c
include/prob2_mpc_export/acado_common.h
include/prob2_mpc_export/acado_integrator.c
include/prob2_mpc_export/acado_qpoases_interface.cpp
include/prob2_mpc_export/acado_qpoases_interface.hpp
include/prob2_mpc_export/acado_solver.c
)
target_include_directories(mpc_solver PUBLIC
include
include/qpoases/INCLUDE/
include/qpoases/SRC/
)

add_executable(ctrl_mpc_sim ctrl_mpc_sim.cpp)
target_link_libraries(ctrl_mpc_sim ${Eigen_LIBRARIES})
target_link_libraries(ctrl_mpc_sim ${PYTHON_LIBRARIES})
target_link_libraries(ctrl_mpc_sim mpc_solver)


add_definitions($(EIGEN_DEFINITIONS))

set(CMAKE_CXX_STANDARD 14)








