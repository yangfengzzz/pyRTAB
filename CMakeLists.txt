cmake_minimum_required(VERSION 3.31)
project(pyRTAB)

include("cmake/add_stub.cmake")

set(CMAKE_CXX_STANDARD 20)

# Try to import all Python components potentially needed by nanobind
find_package(Python 3.8
        REQUIRED COMPONENTS Interpreter Development.Module
        OPTIONAL_COMPONENTS Development.SABIModule)
execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

add_subdirectory(third_party)

####################################################################################################
# We are now ready to compile the actual extension module
nanobind_add_module(
        # Name of the extension
        rtab_ext

        # Build libnanobind statically and merge it into the
        # extension (which itself remains a shared library)
        #
        # If your project builds multiple extensions, you can
        # replace this flag by NB_SHARED to conserve space by
        # reusing a shared libnanobind across libraries
        NB_STATIC

        # Source code goes here
        src/py_rtab_ext.cpp
        src/py_odometry.cpp
        src/py_transform.cpp
        src/py_camera_model.cpp
)

add_stub(
        rtab_stub
        MODULE rtab_ext
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/py_rtab
        PYTHON_PATH $<TARGET_FILE_DIR:rtab_ext>
        DEPENDS rtab_ext
)

target_link_libraries(rtab_ext PRIVATE
        rtabmap_core
)
