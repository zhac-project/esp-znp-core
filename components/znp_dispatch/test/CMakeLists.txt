cmake_minimum_required(VERSION 3.16)
project(znp_dispatch_tests CXX C)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(test_znp_dispatch
    test_znp_dispatch.cpp
    ../znp_dispatch.c
    ../../mt_proto/mt_proto.cpp)
target_include_directories(test_znp_dispatch PRIVATE ../include ../../mt_proto/include)
enable_testing()
add_test(NAME znp_dispatch COMMAND test_znp_dispatch)
