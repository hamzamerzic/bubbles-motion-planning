include_directories(.)

add_library(pqp_environment pqp_environment.cc)
target_link_libraries(pqp_environment
                      PQP
                      dh_parameter
                      model_parser)

add_library(model_parser model_parser.cc)
target_link_libraries(model_parser
                      PQP)

#Tests
add_executable(pqp_environment_test pqp_environment_test.cc)
target_link_libraries(pqp_environment_test
                      pqp_environment
                      naive_generator
                      boost_unit_test_framework)
add_test(PQP_ENVIRONMENT_TEST ${CMAKE_CURRENT_BINARY_DIR}/pqp_environment_test)

