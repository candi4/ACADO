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
include_directories(prob2_mpc_export)
include_directories(prob2_mpc_export/qpoases/SRC)
include_directories(prob2_mpc_export/qpoases/INCLUDE)

add_library(mpc_solver
prob2_mpc_export/qpoases/SRC/Bounds.cpp
prob2_mpc_export/qpoases/SRC/Constraints.cpp
prob2_mpc_export/qpoases/SRC/CyclingManager.cpp
prob2_mpc_export/qpoases/SRC/Indexlist.cpp
prob2_mpc_export/qpoases/SRC/MessageHandling.cpp
prob2_mpc_export/qpoases/SRC/MessageHandling.ipp
prob2_mpc_export/qpoases/SRC/QProblemB.cpp
prob2_mpc_export/qpoases/SRC/QProblem.cpp
prob2_mpc_export/qpoases/SRC/SubjectTo.cpp
prob2_mpc_export/qpoases/SRC/Utils.cpp
prob2_mpc_export/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
prob2_mpc_export/acado_auxiliary_functions.c
prob2_mpc_export/acado_common.h
prob2_mpc_export/acado_integrator.c
prob2_mpc_export/acado_qpoases_interface.cpp
prob2_mpc_export/acado_qpoases_interface.hpp
prob2_mpc_export/acado_solver.c
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








