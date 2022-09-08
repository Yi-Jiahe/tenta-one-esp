set(srcs "main.c"
    "esp_hid_gap.c"
    "hid_cc_report.c")
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
    INCLUDE_DIRS "${include_dirs}"
    REQUIRES esp_hid
    REQUIRES "esp_adc_cal"
    REQUIRES "rotary_encoder")