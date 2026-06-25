set(minmea_SRCS "minmea.c")

# Register sources and public include dir (component's include is current dir)
idf_component_register(SRCS ${minmea_SRCS}
                       INCLUDE_DIRS ".")

# Define MINMEA_INCLUDE_COMPAT for this component only
if(TARGET ${COMPONENT_LIB})
    target_compile_definitions(${COMPONENT_LIB} PUBLIC MINMEA_INCLUDE_COMPAT=0)
else()
    # Fallback: if COMPONENT_LIB isn't set for some reason, use add_compile_definitions
    add_compile_definitions(MINMEA_INCLUDE_COMPAT=0)
endif()