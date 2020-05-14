project(freetype2-sdf C)

# include freetype CMakeLists.txt
add_subdirectory(freetype)

# add the source files
add_library(freetype2-sdf STATIC src/sdfgen.c)

# add compile defination
add_compile_definitions(FT2_BUILD_LIBRARY)

# add freetype include directory
target_include_directories(freetype2-sdf PRIVATE "freetype/include")

# link freetype library
target_link_libraries(freetype2-sdf PUBLIC freetype)