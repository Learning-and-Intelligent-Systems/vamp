# Python bindings configuration for VAMP
# This file contains all Python bindings related logic

if(VAMP_BUILD_PYTHON_BINDINGS)
  # Find Python and nanobind dependencies
  find_package(Python 3.8
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)

  CPMAddPackage("gh:wjakob/nanobind#9a25aed8a7edfe60ef9ad1c911e57667bc4916c4")

  # Check if Python is available
  if(NOT Python_FOUND)
    message(FATAL_ERROR "VAMP_BUILD_PYTHON_BINDINGS is ON but Python was not found")
  endif()

  # Which robots are compiled into the extension. Each robot is one long
  # translation unit (droid and pr2x alone are two thirds of the source), so
  # the set decides the build time. VAMP_ROBOT_SET names one:
  #   default    rainbow, rainbowl (the left hand's attachment frame), spot
  #   extended   default + rainbowcam, rainbowcaml, droid, pandax, fetchx, pr2x, movo
  #   upstream   upstream VAMP's own: sphere, ur5, panda, fetch, baxter
  # An explicit VAMP_ROBOT_MODULES / VAMP_ROBOT_STRUCTS pair overrides it.
  # A robot left out is absent from the Python module (vamp.robots lists what
  # is there); HPN checks such a robot with its exact collision checker.
  if(NOT VAMP_ROBOT_MODULES)
    if(NOT VAMP_ROBOT_SET)
      set(VAMP_ROBOT_SET "default")
    endif()
    set(_vamp_default_modules rainbow rainbowl spot)
    set(_vamp_default_structs Rainbow Rainbowl Spot)
    if(VAMP_ROBOT_SET STREQUAL "default")
      set(VAMP_ROBOT_MODULES ${_vamp_default_modules})
      set(VAMP_ROBOT_STRUCTS ${_vamp_default_structs})
    elseif(VAMP_ROBOT_SET STREQUAL "extended")
      set(VAMP_ROBOT_MODULES ${_vamp_default_modules}
        rainbowcam rainbowcaml droid pandax fetchx pr2x movo)
      set(VAMP_ROBOT_STRUCTS ${_vamp_default_structs}
        RainbowCam RainbowCaml Droid Pandax Fetchx Pr2x Movo)
    elseif(VAMP_ROBOT_SET STREQUAL "upstream")
      set(VAMP_ROBOT_MODULES sphere ur5 panda fetch baxter)
      set(VAMP_ROBOT_STRUCTS Sphere UR5 Panda Fetch Baxter)
    else()
      message(FATAL_ERROR
        "VAMP_ROBOT_SET is '${VAMP_ROBOT_SET}': expected default, extended or upstream")
    endif()
  endif()
  message(STATUS "VAMP robots: ${VAMP_ROBOT_MODULES}")

  foreach(robot ${VAMP_ROBOT_MODULES})
    string(APPEND VAMP_ROBOT_INITS "    vb::init_${robot}(pymodule);\n")
    string(APPEND VAMP_ROBOT_DECLS "    void init_${robot}(nanobind::module_ &pymodule);\n")
    string(APPEND VAMP_ROBOT_QUOTED "\"${robot}\",")
  endforeach()

  list(JOIN VAMP_ROBOT_QUOTED ", " VAMP_ROBOT_NAMES)

  configure_file(
    src/impl/vamp/bindings/python/init.hh.in
    ${CMAKE_CURRENT_BINARY_DIR}/vamp_python_init.hh
    @ONLY
  )

  configure_file(
    src/impl/vamp/bindings/python/python.cc.in
    ${CMAKE_CURRENT_BINARY_DIR}/python.cc
    @ONLY
  )

  list(APPEND VAMP_EXT_SOURCES
    src/impl/vamp/bindings/python/environment.cc
    src/impl/vamp/bindings/python/settings.cc
    ${CMAKE_CURRENT_BINARY_DIR}/python.cc
  )
  foreach(robot_name robot_struct IN ZIP_LISTS VAMP_ROBOT_MODULES VAMP_ROBOT_STRUCTS)
  configure_file(
    src/impl/vamp/bindings/python/robot.cc.in
    ${CMAKE_CURRENT_BINARY_DIR}/${robot_name}.cc
    @ONLY
  )

  list(APPEND VAMP_EXT_SOURCES
    ${CMAKE_CURRENT_BINARY_DIR}/${robot_name}.cc
  )
  endforeach()

  nanobind_add_module(_core_ext
    NB_STATIC
    STABLE_ABI
    NOMINSIZE
    ${VAMP_EXT_SOURCES}
  )

  target_include_directories(_core_ext
    SYSTEM PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
  )

  target_link_libraries(_core_ext
    PRIVATE
    vamp_cpp
    Eigen3::Eigen
  )

  if($ENV{GITHUB_ACTIONS})
    set(STUB_PREFIX "")
  else()
    set(STUB_PREFIX "${CMAKE_BINARY_DIR}/stubs/")
  endif()

  # Disable strict warnings for Python bindings to maintain compatibility
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(_core_ext PRIVATE -Wno-c++11-narrowing -Wno-sign-compare)
  endif()

  nanobind_add_stub(
    vamp_stub
    MODULE _core_ext
    OUTPUT "${STUB_PREFIX}__init__.pyi"
    PYTHON_PATH $<TARGET_FILE_DIR:_core_ext>
    DEPENDS _core_ext
    VERBOSE
  )

  foreach(robot_name IN LISTS VAMP_ROBOT_MODULES)
    nanobind_add_stub(
      "vamp_${robot_name}_stub"
      MODULE "_core_ext.${robot_name}"
      OUTPUT "${STUB_PREFIX}${robot_name}.pyi"
      PYTHON_PATH $<TARGET_FILE_DIR:_core_ext>
      DEPENDS _core_ext
      VERBOSE
    )
  endforeach()

  install(
    TARGETS _core_ext
    LIBRARY
    DESTINATION vamp/_core
  )

  install(
    FILES "${STUB_PREFIX}__init__.pyi"
    DESTINATION "${CMAKE_SOURCE_DIR}/src/vamp/_core"
  )

  foreach(robot_name IN LISTS VAMP_ROBOT_MODULES)
    install(
      FILES "${STUB_PREFIX}${robot_name}.pyi"
      DESTINATION "${CMAKE_SOURCE_DIR}/src/vamp/_core"
    )
  endforeach()
endif() 
