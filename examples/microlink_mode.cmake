set(MICROLINK_MODE "uart" CACHE STRING "MicroLink modem variant to use (uart or usb)")
set_property(CACHE MICROLINK_MODE PROPERTY STRINGS uart usb)

get_filename_component(_MICROLINK_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

if(MICROLINK_MODE STREQUAL "uart")
    set(EXTRA_COMPONENT_DIRS "${_MICROLINK_REPO_ROOT}/microlink-mode-uart")
elseif(MICROLINK_MODE STREQUAL "usb")
    set(EXTRA_COMPONENT_DIRS "${_MICROLINK_REPO_ROOT}/microlink-mode-usb")
else()
    message(FATAL_ERROR "Unsupported MICROLINK_MODE='${MICROLINK_MODE}'. Expected 'uart' or 'usb'.")
endif()
