set(PROJECT_NAME "epever-esp-idf")
set(srcs 
    "epever.c" 
    "main.c"
)

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "."
)

