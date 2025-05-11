idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
)

# enable modern C++ for just this component
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)
