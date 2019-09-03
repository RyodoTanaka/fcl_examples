find_package(PkgConfig)

pkg_check_modules(FCL fcl>=0.5.0 REQUIRED)

include(FindPackageHandleStandardArgs)
# if all listed variables are TRUE
find_package_handle_standard_args(FCL DEFAULT_MSG
  FCL_LIBRARIES FCL_INCLUDE_DIRS)
mark_as_advanced(FCL_INCLUDE_DIRS FCL_LIBRARIES)

if(${FCL_FOUND})
  message(STATUS "Found FCL version: " ${FCL_VERSION} " installed in: " ${FCL_PREFIX})
else()
  message(SEND_ERROR "Could not find FCL")
endif()
