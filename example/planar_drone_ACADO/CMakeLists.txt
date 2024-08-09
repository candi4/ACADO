cmake_minimum_required(VERSION 3.10)
project(planar_drone_20)


find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(MPC_SOLVER)
include_directories(MPC_SOLVER/qpoases/SRC)
include_directories(MPC_SOLVER/qpoases/INCLUDE)

add_library(mpc_solver
MPC_SOLVER/qpoases/SRC/Bounds.cpp
MPC_SOLVER/qpoases/SRC/Constraints.cpp
MPC_SOLVER/qpoases/SRC/CyclingManager.cpp
MPC_SOLVER/qpoases/SRC/Indexlist.cpp
MPC_SOLVER/qpoases/SRC/MessageHandling.cpp
MPC_SOLVER/qpoases/SRC/MessageHandling.ipp
MPC_SOLVER/qpoases/SRC/QProblemB.cpp
MPC_SOLVER/qpoases/SRC/QProblem.cpp
MPC_SOLVER/qpoases/SRC/SubjectTo.cpp
MPC_SOLVER/qpoases/SRC/Utils.cpp
MPC_SOLVER/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
MPC_SOLVER/acado_auxiliary_functions.c
MPC_SOLVER/acado_common.h
MPC_SOLVER/acado_integrator.c
MPC_SOLVER/acado_qpoases_interface.cpp
MPC_SOLVER/acado_qpoases_interface.hpp
MPC_SOLVER/acado_solver.c
)
target_include_directories(mpc_solver PUBLIC
MPC_SOLVER
MPC_SOLVER/qpoases/INCLUDE/
MPC_SOLVER/qpoases/SRC/
)

add_executable(planar_drone_20 planar_drone_20.cpp)
target_link_libraries(planar_drone_20 ${Eigen_LIBRARIES})
target_link_libraries(planar_drone_20 ${PYTHON_LIBRARIES})
target_link_libraries(planar_drone_20 mpc_solver)


add_definitions($(EIGEN_DEFINITIONS))

set(CMAKE_CXX_STANDARD 14)