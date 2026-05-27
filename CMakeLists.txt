idf_component_register(
  SRCS
    "src/TinyGPS++.cpp"
    "src/unicore.cpp"
  INCLUDE_DIRS
    "src"
  REQUIRES
    esp_timer
)

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 26)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)
