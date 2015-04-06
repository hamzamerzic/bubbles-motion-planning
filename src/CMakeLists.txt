include_directories(.)

add_subdirectory(PQP)
add_subdirectory(debugging)
add_subdirectory(environment)
add_subdirectory(random_generator)

add_library(dh_parameter dh_parameter.cc)
target_link_libraries(dh_parameter
                      PQP)

add_library(simple_tree simple_tree.cc)
target_link_libraries(simple_tree
                      pqp_environment)

add_executable(main main.cc)
target_link_libraries(main
                      PQP)

#Tests
add_executable(dh_parameter_test dh_parameter_test.cc)
target_link_libraries(dh_parameter_test
                      dh_parameter
                      boost_unit_test_framework)
add_test(DH_PARAMETER_TEST ${CMAKE_CURRENT_BINARY_DIR}/dh_parameter_test)

add_executable(simple_tree_test simple_tree_test.cc)
target_link_libraries(simple_tree_test
                      simple_tree
                      pqp_environment
                      naive_generator
                      boost_unit_test_framework)
add_test(SIMPLE_TREE_TEST ${CMAKE_CURRENT_BINARY_DIR}/simple_tree_test)
