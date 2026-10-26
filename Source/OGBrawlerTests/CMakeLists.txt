# SPDX-License-Identifier: BUSL-1.1

file(GLOB_RECURSE OGBRAWLER_TESTS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

if(NOT TARGET og_simulation_jolt)
    list(FILTER OGBRAWLER_TESTS_SOURCES EXCLUDE REGEX "/GoldenTrace/")
endif()

add_executable(og_brawler_tests ${OGBRAWLER_TESTS_SOURCES})

target_compile_definitions(og_brawler_tests PRIVATE
    WITH_LOW_LEVEL_TESTS=1
)

target_link_libraries(og_brawler_tests PRIVATE og_brawler og_simulation Catch2 glm::glm-header-only)

if(TARGET og_simulation_jolt)
    target_link_libraries(og_brawler_tests PRIVATE og_simulation_jolt)
endif()

add_test(NAME og_brawler_tests COMMAND og_brawler_tests)
