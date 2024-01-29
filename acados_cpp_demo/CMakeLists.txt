cmake_minimum_required(VERSION 3.1)
project(nmpc_caller LANGUAGES C CXX)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

# for macOS
set(CMAKE_MACOSX_RPATH 1)

# set compiler standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")

# include Eigen3
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${Eigen3_INCLUDE_DIRS})

# include some more paths
include_directories("./include")
include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)
include_directories(../acados_python_demo/c_generated_code)

# load acados libs
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)

file(GLOB ocp_solver
        ../acados_python_demo/c_generated_code/acados_solver_srbd.c
        ../acados_python_demo/c_generated_code/srbd_constraints/srbd_constr_h_fun_jac_uxt_zt.c
        ../acados_python_demo/c_generated_code/srbd_constraints/srbd_constr_h_fun.c
        ../acados_python_demo/c_generated_code/srbd_model/srbd_expl_vde_adj.c
        ../acados_python_demo/c_generated_code/srbd_model/srbd_expl_ode_fun.c
        ../acados_python_demo/c_generated_code/srbd_model/srbd_expl_vde_forw.c
        )

# create libraries
add_library(ocp_shared_lib SHARED ${ocp_solver})
target_link_libraries(ocp_shared_lib acados hpipm blasfeo)

add_executable(nmpc_caller src/nmpc_caller.cpp src/main.cpp)
target_link_libraries(nmpc_caller ocp_shared_lib)