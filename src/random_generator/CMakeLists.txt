include_directories(.)

add_library(naive_generator naive_generator.cc)
target_link_libraries(naive_generator)

add_library(halton_generator halton_generator.cc)
target_link_libraries(halton_generator)