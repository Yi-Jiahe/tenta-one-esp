set(srcs "main.c"
    "hid_dev.c"
    "esp_hidd_prf_api.c"
    "hid_device_le_prf.c")
set(include_dirs ".")
set(requires 
    "esp_hid"
    "esp_adc_cal"
    "esp_lcd"
    "rotary_encoder")
idf_component_register(SRCS "${srcs}"
    INCLUDE_DIRS "${include_dirs}"
    REQUIRES "${requires}")