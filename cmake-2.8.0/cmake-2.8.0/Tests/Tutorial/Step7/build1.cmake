SET(CTEST_SOURCE_DIRECTORY "$ENV{HOME}/Dashboards/My Tests/CMake/Tests/Tutorial/Step7")
SET(CTEST_BINARY_DIRECTORY "${CTEST_SOURCE_DIRECTORY}-build1")

SET(CTEST_CMAKE_COMMAND "cmake")
SET(CTEST_COMMAND "ctest -D Experimental")
