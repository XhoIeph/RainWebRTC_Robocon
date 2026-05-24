set(APPS_DIR ./)

# P0: Only compile camera pipeline files (no LVGL/Brookesia UI)
file(GLOB_RECURSE APPS_C_SRCS ${APPS_DIR}/camera/*.c)
file(GLOB_RECURSE APPS_CPP_SRCS ${APPS_DIR}/camera/*.cpp)

# Exclude LVGL-dependent and AI-dependent files
list(FILTER APPS_C_SRCS EXCLUDE REGEX "camera/assets/")
list(FILTER APPS_C_SRCS EXCLUDE REGEX "camera/ui/")
list(FILTER APPS_CPP_SRCS EXCLUDE REGEX "camera/ui/")
list(FILTER APPS_CPP_SRCS EXCLUDE REGEX "Camera\\.cpp$")
list(FILTER APPS_CPP_SRCS EXCLUDE REGEX "pedestrian_detect")
list(FILTER APPS_CPP_SRCS EXCLUDE REGEX "humanface_detect")
list(FILTER APPS_CPP_SRCS EXCLUDE REGEX "camera_pipeline")

idf_component_register(
    SRCS ${APPS_C_SRCS} ${APPS_CPP_SRCS}
    INCLUDE_DIRS ${APPS_DIR}
    REQUIRES esp_event esp_wifi nvs_flash esp_driver_jpeg esp_mm wtdkp4c5_s1_board esp_video
)

target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
    $<$<COMPILE_LANGUAGE:C>:
        -Wno-format
        -Wno-incompatible-pointer-types
        -Wno-int-conversion
    >
    $<$<COMPILE_LANGUAGE:CXX>:
        -Wno-format
        -Wno-conversion-null
        -Wno-attributes
        -Wno-write-strings
    >
)
