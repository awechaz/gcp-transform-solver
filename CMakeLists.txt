cmake_minimum_required(VERSION 3.16...3.27)

project(GCPTransformSolver)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${GCPTransformSolver_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${GCPTransformSolver_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${GCPTransformSolver_BINARY_DIR}/lib)

add_subdirectory(src)
