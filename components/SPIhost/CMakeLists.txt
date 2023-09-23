#idf_component_register(SRCS "SPIhost.cpp"
#                    INCLUDE_DIRS "include")

set(COMPONENT_SRCDIRS ".")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_PRIV_REQUIRES  "driver")
register_component()  