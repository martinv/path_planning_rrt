add_executable(
    unit_tests
    PolygonTest.cpp
    GraphIOTest.cpp
)

target_link_libraries(
    unit_tests
    geo
    utils
    fmt
    GTest::gtest_main
    GTest::gtest
    GTest::gmock
)

include(GoogleTest)
gtest_discover_tests(unit_tests)

