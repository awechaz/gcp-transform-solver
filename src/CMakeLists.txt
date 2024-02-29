find_package(Ceres REQUIRED)

add_executable(point_coordinate_solver point_coordinate_solver.cc)
target_link_libraries(point_coordinate_solver PRIVATE ${CERES_LIBRARIES})

add_executable(test_gcp_transform test_gcp_transform.cc)
target_link_libraries(test_gcp_transform PRIVATE ${CERES_LIBRARIES})
