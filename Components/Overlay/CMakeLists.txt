#-------------------------------------------------------------------
# This file is part of the CMake build system for OGRE-Next
#     (Object-oriented Graphics Rendering Engine)
# For the latest info, see http://www.ogre3d.org/
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

############################################################
# Overlay optional component
############################################################

PROJECT(OgreOverlay)

# define header and source files for the library
file(GLOB HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# Add needed definitions 
add_definitions(-DOGRE_OVERLAY_EXPORTS -D_MT -D_USRDLL)

# include headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OGRE_SOURCE_DIR}/OgreMain/include)

ogre_add_component_include_dir(Hlms/Unlit)
#ogre_add_component_include_dir(Hlms/UnlitMobile)
ogre_add_component_include_dir(Hlms/Common)

# setup target
ogre_add_library(OgreOverlay ${OGRE_COMP_LIB_TYPE} ${HEADER_FILES} ${SOURCE_FILES} ${PLATFORM_HEADER_FILES} ${PLATFORM_SOURCE_FILES})
set_target_properties(OgreOverlay PROPERTIES VERSION ${OGRE_SOVERSION} SOVERSION ${OGRE_SOVERSION})
target_link_libraries(OgreOverlay OgreMain ${FREETYPE_LIBRARIES} OgreHlmsUnlit )
if (OGRE_CONFIG_THREADS)
  target_link_libraries(OgreOverlay ${OGRE_THREAD_LIBRARIES})
endif ()

# install 
ogre_config_framework(OgreOverlay)
ogre_config_component(OgreOverlay)

install(FILES ${HEADER_FILES}
  DESTINATION include/OGRE/Overlay
)

