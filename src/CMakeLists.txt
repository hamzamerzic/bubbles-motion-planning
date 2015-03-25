include_directories(.)
add_subdirectory(PQP)
add_subdirectory(random_generator)
add_subdirectory(debugging)

add_library(dh_parameter dh_parameter.cc)
target_link_libraries(dh_parameter)

add_library(pqp_environment pqp_environment.cc)
target_link_libraries(pqp_environment
                      PQP
                      dh_parameter)

add_executable(main main.cc)
target_link_libraries(main
                      PQP)

#Tests
add_executable(dh_parameter_test dh_parameter_test.cc)
target_link_libraries(dh_parameter_test
                      dh_parameter
		                  boost_unit_test_framework)
add_test(DH_PARAMETER_TEST ${CMAKE_CURRENT_BINARY_DIR}/dh_parameter_test)

add_executable(pqp_environment_test pqp_environment_test.cc)
target_link_libraries(pqp_environment_test
                      pqp_environment
                      naive_generator
                      boost_unit_test_framework)
add_test(DH_PARAMETER_TEST ${CMAKE_CURRENT_BINARY_DIR}/pqp_environment_test)
