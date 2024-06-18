
include_directories(../contrib/gtest)
add_executable(utest 
                utest.cpp 
                ui/IO.cpp 
                ui/DataFilters.cpp 
                ui/Matcher.cpp 
                ui/Outliers.cpp 
                ui/ErrorMinimizers.cpp 
                ui/Transformations.cpp  
                ui/DataPoints.cpp 
                ui/Inspectors.cpp 
                ui/Loggers.cpp
                ui/octree/Octree.cpp
                ui/icp/GeneralTests.cpp
                ui/utils_filesystem.cpp
                ui/utils_geometry.cpp
                ui/utils_gtest.cpp
                ui/utils_transformations.cpp
                ui/TransformationError.cpp
                ui/icp/RegistrationTestCase.cpp
                ui/icp/RegistrationTestResult.cpp
                ui/icp/utils_registration.cpp
                ui/icp/Conditioning.cpp
                )

find_package (Threads)
target_link_libraries(utest gtest pointmatcher ${CMAKE_THREAD_LIBS_INIT})

add_test(utest ${CMAKE_CURRENT_BINARY_DIR}/utest --path "${CMAKE_SOURCE_DIR}/examples/data/")

