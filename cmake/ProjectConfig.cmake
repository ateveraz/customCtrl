set(UAV_PROJECT_NAME "template"
    CACHE STRING "UAV application name and FlAIR installation directory")

if(NOT UAV_PROJECT_NAME MATCHES "^[A-Za-z][A-Za-z0-9_-]*$")
    message(FATAL_ERROR
        "Invalid UAV_PROJECT_NAME: '${UAV_PROJECT_NAME}'. "
        "Use a letter followed by letters, digits, '_' or '-'.")
endif()

set(UAV_EXECUTABLE_NAME "${UAV_PROJECT_NAME}")
set(SIMULATOR_EXECUTABLE_NAME "${UAV_PROJECT_NAME}_simulator")
set(FLAIR_DEMO_DIRECTORY "${UAV_PROJECT_NAME}")