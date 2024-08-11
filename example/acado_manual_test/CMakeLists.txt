cmake_minimum_required(VERSION 3.10)
project(acado_manual_test)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(ACADO REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(. ${ACADO_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

set(SOURCE_FILES
    src/chapter3/ocp.cpp
    src/chapter3/ocp_initialize_with_text.cpp
    src/chapter3/ocp_initialize_with_datastructure.cpp
    src/chapter3/ocp_option_change.cpp
    src/chapter3/ocp_logging.cpp
    src/chapter3/ocp_storing2text.cpp
    src/chapter3/ocp_storing2datastructure.cpp
    src/chapter3/discrete_time_optimization.cpp
    src/chapter3/dae_optimization.cpp

    src/chapter4/scalar2_nbi.cpp
    src/chapter4/scalar2_nnc.cpp
    src/chapter4/scalar2_ws.cpp
    src/chapter4/scalar3_nbi.cpp
    src/chapter4/scalar3_nnc.cpp
    src/chapter4/scalar3_ws.cpp
    src/chapter4/plug_flow_reactor_nbi.cpp
    src/chapter4/plug_flow_reactor_nnc.cpp
    src/chapter4/plug_flow_reactor_ws.cpp

    src/chapter5/parameter_estimation.cpp
    
    src/chapter6/process.cpp
    src/chapter6/process_advanced.cpp

    src/chapter7/controller.cpp
    src/chapter7/controller_classical.cpp

    src/chapter8/simulation_mpc_controller.cpp
    src/chapter8/simulation_classical_controller.cpp

)

function(create_executable target_name source_path)
    add_executable(${target_name} ${source_path})
    target_link_libraries(${target_name} ${ACADO_SHARED_LIBRARIES} ${Eigen_LIBRARIES} ${PYTHON_LIBRARIES})

    get_filename_component(source_dir ${source_path} DIRECTORY)

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${target_name}> ${CMAKE_SOURCE_DIR}/${source_dir}
        COMMENT "Copying executable to ${CMAKE_SOURCE_DIR}/${source_dir}"
    )
endfunction()

# chapter3
create_executable(ocp src/chapter3/ocp.cpp)
create_executable(ocp_initialize_with_text src/chapter3/ocp_initialize_with_text.cpp)
create_executable(ocp_initialize_with_datastructure src/chapter3/ocp_initialize_with_datastructure.cpp)
create_executable(ocp_option_change src/chapter3/ocp_option_change.cpp)
create_executable(ocp_logging src/chapter3/ocp_logging.cpp)
create_executable(ocp_storing2text src/chapter3/ocp_storing2text.cpp)
create_executable(ocp_storing2datastructure src/chapter3/ocp_storing2datastructure.cpp)
create_executable(discrete_time_optimization src/chapter3/discrete_time_optimization.cpp)
create_executable(dae_optimization src/chapter3/dae_optimization.cpp)

# chapter4
create_executable(scalar2_nbi src/chapter4/scalar2_nbi.cpp)
create_executable(scalar2_nnc src/chapter4/scalar2_nnc.cpp)
create_executable(scalar2_ws src/chapter4/scalar2_ws.cpp)
create_executable(scalar3_nbi src/chapter4/scalar3_nbi.cpp)
create_executable(scalar3_nnc src/chapter4/scalar3_nnc.cpp)
create_executable(scalar3_ws src/chapter4/scalar3_ws.cpp)
create_executable(plug_flow_reactor_nbi src/chapter4/plug_flow_reactor_nbi.cpp)
create_executable(plug_flow_reactor_nnc src/chapter4/plug_flow_reactor_nnc.cpp)
create_executable(plug_flow_reactor_ws src/chapter4/plug_flow_reactor_ws.cpp)

# chapter5
create_executable(parameter_estimation src/chapter5/parameter_estimation.cpp)

# chapter6
create_executable(process src/chapter6/process.cpp)
create_executable(process_advanced src/chapter6/process_advanced.cpp)

# chapter7
create_executable(controller src/chapter7/controller.cpp)
create_executable(controller_classical src/chapter7/controller_classical.cpp)

# chapter8
create_executable(simulation_mpc_controller src/chapter8/simulation_mpc_controller.cpp)
create_executable(simulation_classical_controller src/chapter8/simulation_classical_controller.cpp)

add_definitions(${EIGEN_DEFINITIONS})

set(CMAKE_CXX_STANDARD 14)
