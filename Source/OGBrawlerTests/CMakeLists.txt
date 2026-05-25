# SPDX-License-Identifier: BUSL-1.1

file(GLOB_RECURSE OGBRAWLER_TESTS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

add_executable(og_brawler_tests ${OGBRAWLER_TESTS_SOURCES})

target_compile_definitions(og_brawler_tests PRIVATE
    WITH_LOW_LEVEL_TESTS=1
)

target_link_libraries(og_brawler_tests PRIVATE og_brawler og_simulation Catch2 glm::glm-header-only)

add_test(NAME og_brawler_tests COMMAND og_brawler_tests)
