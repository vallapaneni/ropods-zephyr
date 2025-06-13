# CMakeLists.txt for lib directory

# Add sensor manager library
add_subdirectory(src)

# Add other libraries as needed
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/emd/CMakeLists.txt)
  add_subdirectory(emd)
endif()
