# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)
idf_component_register(SRCS ${app_sources})

set(COMPONENT_ADD_INCLUDEDIRS ".")

#register_component()
target_add_binary_data(${COMPONENT_TARGET} "certs/aws-root-ca.pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/certificate.pem.crt" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "certs/private.pem.key" TEXT)
