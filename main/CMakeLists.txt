set(srcs "main.c"
    "esp_hid_gap.c"
    "hid_cc_report.c")
set(include_dirs ".")
set(requires esp_hid
    "esp_adc_cal"
    "rotary_encoder")
idf_component_register(SRCS "${srcs}"
    INCLUDE_DIRS "${include_dirs}"
    REQUIRES "${requires}")