add_library(stb_image src/stb_image.cpp)
target_include_directories(stb_image PUBLIC include/ PRIVATE include/stb_image)

install(TARGETS stb_image
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
