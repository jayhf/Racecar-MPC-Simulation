project(carsim)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig  REQUIRED)
pkg_check_modules(GTKMM gtkmm-3.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/autonomous/carsim/modules")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native -DEIGEN_DONT_VECTORIZE")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -march=native -DEIGEN_DONT_VECTORIZE")

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLM REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(EIGEN3 REQUIRED)
find_package(JPEG REQUIRED)
find_package(GUROBI REQUIRED)

include_directories(
        .
        ${OPENGL_INCLUDE_DIRS}
        ${JPEG_INCLUDE_DIRS}
        ${GTKMM_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
        ${ASSIMP_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${GUROBI_INCLUDE_DIRS}
)

link_directories(
        ${GTKMM_LIBRARY_DIRS}
        ${ASSIMP_LIBRARY_DIRS}
        ${EIGEN3_LIBRARY_DIRS}
)

#This shouldn't be necessary, but the program won't compile without the extra source files...
file(GLOB_RECURSE GUROBI_SOURCE
        "C:/gurobi810/win64/src/cpp/*.cpp")

add_executable(carsim main.cpp RgbImage.cpp GlModel.cpp Simulation.cpp Path.cpp OpenGlWrappers.cpp RaceWindow.cpp
        ${GUROBI_SOURCE})

target_link_libraries(carsim
        ${OPENGL_LIBRARIES}
        ${GTKMM_LIBRARIES}
        ${ASSIMP_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${GUROBI_LIBRARIES})