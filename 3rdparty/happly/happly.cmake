include(ExternalProject)

ExternalProject_Add(
  external_happly
  PREFIX happly
  GIT_REPOSITORY https://github.com/nmwsharp/happly.git 
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "")

ExternalProject_Get_Property(external_happly SOURCE_DIR)
add_library(libhapplyHelper INTERFACE)
add_dependencies(libhapplyHelper external_happly)
target_include_directories(libhapplyHelper SYSTEM INTERFACE $<BUILD_INTERFACE:${SOURCE_DIR}>)
set_property(TARGET libhapplyHelper PROPERTY EXPORT_NAME happly3::happly)
add_library(happly ALIAS libhapplyHelper)
