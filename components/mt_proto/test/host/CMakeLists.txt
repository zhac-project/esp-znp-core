# SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
# SPDX-License-Identifier: AGPL-3.0-or-later
#
# Host-side cross-repo wire-constant drift guard (def 5 / P6-T33).
cmake_minimum_required(VERSION 3.16)
project(mt_proto_wire_constants_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_wire_constants
    test_wire_constants.cpp
    ../../mt_proto.cpp)
target_include_directories(test_wire_constants PRIVATE ../../include)

enable_testing()
add_test(NAME mt_proto_wire_constants COMMAND test_wire_constants)
