set(src_dirs "main" "radio" "platform" "sec" "mac" "mac/region")
set(inc_dirs "radio" "platform" "sec" "mac" "mac/region")
set(defines)

# Always enable ISM2400
list(APPEND src_dirs "mac/region/ISM2400")
list(APPEND inc_dirs "mac/region/ISM2400")
list(APPEND defines REGION_ISM2400)

# Add selected sub-GHz region
if(CONFIG_LORAWAN_REGION_EU868)
  list(APPEND src_dirs "mac/region/EU868")
  list(APPEND inc_dirs "mac/region/EU868")
  list(APPEND defines REGION_EU868)
endif()

if(CONFIG_LORAWAN_REGION_US915)
  list(APPEND src_dirs "mac/region/US915")
  list(APPEND inc_dirs "mac/region/US915")
  list(APPEND defines REGION_US915)
endif()

#
idf_component_register(SRC_DIRS "${src_dirs}"
                    INCLUDE_DIRS "${inc_dirs}")

target_compile_definitions(${COMPONENT_LIB} PUBLIC SOFT_SE=1 ${defines})
