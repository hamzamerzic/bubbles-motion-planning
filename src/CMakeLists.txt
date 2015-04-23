include_directories(.)

add_subdirectory(PQP)
add_subdirectory(environment)
add_subdirectory(models)
add_subdirectory(random_generator)

add_library(dh_parameter dh_parameter.cc)
target_link_libraries(dh_parameter
                      PQP)

add_library(two_seg_prm two_seg_prm.cc)
target_link_libraries(two_seg_prm
                      pqp_environment)

add_executable(main main.cc)
target_link_libraries(main
                      PQP)

configure_file(cylinder.stl cylinder.stl COPYONLY)

#Tests
add_executable(dh_parameter_test dh_parameter_test.cc)
target_link_libraries(dh_parameter_test
                      dh_parameter
                      boost_unit_test_framework)
add_test(DH_PARAMETER_TEST ${CMAKE_CURRENT_BINARY_DIR}/dh_parameter_test)

add_executable(two_seg_prm_test two_seg_prm_test.cc)
target_link_libraries(two_seg_prm_test
                      two_seg_prm
                      pqp_environment
                      naive_generator
                      boost_unit_test_framework)
add_test(SIMPLE_TREE_TEST ${CMAKE_CURRENT_BINARY_DIR}/two_seg_prm_test)