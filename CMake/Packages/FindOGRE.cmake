#-------------------------------------------------------------------
# This file is part of the CMake build system for OGRE-Next
#     (Object-oriented Graphics Rendering Engine)
# For the latest info, see http://www.ogre3d.org/
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

# - Try to find OGRE
# If you have multiple versions of Ogre installed, use the CMake or
# the environment variable OGRE_HOME to point to the path where the
# desired Ogre version can be found.
# By default this script will look for a dynamic Ogre build. If you
# need to link against static Ogre libraries, set the CMake variable
# OGRE_STATIC to TRUE.
#
# Once done, this will define
#
#  OGRE_FOUND - system has OGRE
#  OGRE_INCLUDE_DIRS - the OGRE include directories 
#  OGRE_LIBRARIES - link these to use the OGRE core
#  OGRE_BINARY_REL - location of the main Ogre binary (win32 non-static only, release)
#  OGRE_BINARY_DBG - location of the main Ogre binaries (win32 non-static only, debug)
#
# Additionally this script searches for the following optional
# parts of the Ogre package:
#  Plugin_CgProgramManager, Plugin_ParticleFX, 
#  RenderSystem_GL, RenderSystem_GL3Plus,
#  RenderSystem_GLES, RenderSystem_GLES2,
#  RenderSystem_Direct3D9, RenderSystem_Direct3D11
#  Paging, Terrain, Volume, Overlay
#
# For each of these components, the following variables are defined:
#
#  OGRE_${COMPONENT}_FOUND - ${COMPONENT} is available
#  OGRE_${COMPONENT}_INCLUDE_DIRS - additional include directories for ${COMPONENT}
#  OGRE_${COMPONENT}_LIBRARIES - link these to use ${COMPONENT} 
#  OGRE_${COMPONENT}_BINARY_REL - location of the component binary (win32 non-static only, release)
#  OGRE_${COMPONENT}_BINARY_DBG - location of the component binary (win32 non-static only, debug)
#
# Finally, the following variables are defined:
#
#  OGRE_PLUGIN_DIR_REL - The directory where the release versions of
#       the OGRE plugins are located
#  OGRE_PLUGIN_DIR_DBG - The directory where the debug versions of
#       the OGRE plugins are located
#  OGRE_MEDIA_DIR - The directory where the OGRE sample media is
#       located, if available

include(FindPkgMacros)
include(PreprocessorUtils)
findpkg_begin(OGRE)


# Get path, convert backslashes as ${ENV_${var}}
getenv_path(OGRE_HOME)
getenv_path(OGRE_SDK)
getenv_path(OGRE_SOURCE)
getenv_path(OGRE_BUILD)
getenv_path(OGRE_DEPENDENCIES_DIR)
getenv_path(PROGRAMFILES)

# Determine whether to search for a dynamic or static build
if (OGRE_STATIC)
  set(OGRE_LIB_SUFFIX "Static")
else ()
  set(OGRE_LIB_SUFFIX "")
endif ()

if(APPLE AND NOT OGRE_STATIC)
	set(OGRE_LIBRARY_NAMES "Ogre${OGRE_LIB_SUFFIX}")
else()
    set(OGRE_LIBRARY_NAMES "OgreMain${OGRE_LIB_SUFFIX}")
endif()
get_debug_names(OGRE_LIBRARY_NAMES)
          
# construct search paths from environmental hints and
# OS specific guesses
if (WIN32)
  set(OGRE_PREFIX_GUESSES
    ${ENV_PROGRAMFILES}/OGRE
    C:/OgreSDK
  )
elseif (UNIX)
  set(OGRE_PREFIX_GUESSES
    /opt/ogre
    /opt/OGRE
    /usr/lib${LIB_SUFFIX}/ogre
    /usr/lib${LIB_SUFFIX}/OGRE
    /usr/local/lib${LIB_SUFFIX}/ogre
    /usr/local/lib${LIB_SUFFIX}/OGRE
    $ENV{HOME}/ogre
    $ENV{HOME}/OGRE
  )
  if (APPLE)
    set(OGRE_PREFIX_GUESSES 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE}
      ${OGRE_PREFIX_GUESSES}
    )
  endif ()
endif ()
set(OGRE_PREFIX_PATH
  ${OGRE_HOME} ${OGRE_SDK} ${ENV_OGRE_HOME} ${ENV_OGRE_SDK}
  ${OGRE_PREFIX_GUESSES}
)
create_search_paths(OGRE)
# If both OGRE_BUILD and OGRE_SOURCE are set, prepare to find Ogre in a build dir
set(OGRE_PREFIX_SOURCE ${OGRE_SOURCE} ${ENV_OGRE_SOURCE})
set(OGRE_PREFIX_BUILD ${OGRE_BUILD} ${ENV_OGRE_BUILD})
set(OGRE_PREFIX_DEPENDENCIES_DIR ${OGRE_DEPENDENCIES_DIR} ${ENV_OGRE_DEPENDENCIES_DIR})
if (OGRE_PREFIX_SOURCE AND OGRE_PREFIX_BUILD)
  foreach(dir ${OGRE_PREFIX_SOURCE})
    set(OGRE_INC_SEARCH_PATH ${dir}/OgreMain/include ${dir}/Dependencies/include ${dir}/iOSDependencies/include ${dir}/AndroidDependencies/include ${OGRE_INC_SEARCH_PATH})
    set(OGRE_LIB_SEARCH_PATH ${dir}/lib ${dir}/Dependencies/lib ${dir}/iOSDependencies/lib ${dir}/AndroidDependencies/lib/${ANDROID_ABI} ${OGRE_LIB_SEARCH_PATH})
    set(OGRE_BIN_SEARCH_PATH ${dir}/Samples/Common/bin ${OGRE_BIN_SEARCH_PATH})
  endforeach(dir)
  foreach(dir ${OGRE_PREFIX_BUILD})
    set(OGRE_INC_SEARCH_PATH ${dir}/include ${OGRE_INC_SEARCH_PATH})
    if(APPLE AND NOT OGRE_BUILD_PLATFORM_APPLE_IOS)
        set(OGRE_LIB_SEARCH_PATH ${dir}/lib/macosx ${OGRE_LIB_SEARCH_PATH})
    else()
        set(OGRE_LIB_SEARCH_PATH ${dir}/lib ${OGRE_LIB_SEARCH_PATH})
    endif()

    if (OGRE_BUILD_PLATFORM_APPLE_IOS)
        set(OGRE_LIB_SEARCH_PATH ${dir}/lib/iphoneos ${dir}/lib/iphonesimulator ${OGRE_LIB_SEARCH_PATH})
    endif()

    set(OGRE_BIN_SEARCH_PATH ${dir}/bin ${OGRE_BIN_SEARCH_PATH})
    set(OGRE_BIN_SEARCH_PATH ${dir}/Samples/Common/bin ${OGRE_BIN_SEARCH_PATH})

    if(APPLE AND NOT OGRE_BUILD_PLATFORM_APPLE_IOS)
      set(OGRE_BIN_SEARCH_PATH ${dir}/bin/macosx ${OGRE_BIN_SEARCH_PATH})
    endif()
  endforeach(dir)
  
  if (OGRE_PREFIX_DEPENDENCIES_DIR)
    set(OGRE_INC_SEARCH_PATH ${OGRE_PREFIX_DEPENDENCIES_DIR}/include ${OGRE_INC_SEARCH_PATH})
    set(OGRE_LIB_SEARCH_PATH ${OGRE_PREFIX_DEPENDENCIES_DIR}/lib ${OGRE_LIB_SEARCH_PATH})
    set(OGRE_BIN_SEARCH_PATH ${OGRE_PREFIX_DEPENDENCIES_DIR}/bin ${OGRE_BIN_SEARCH_PATH})
  endif()
else()
  set(OGRE_PREFIX_SOURCE "NOTFOUND")
  set(OGRE_PREFIX_BUILD "NOTFOUND")
endif ()

# redo search if any of the environmental hints changed
set(OGRE_COMPONENTS Paging Terrain Volume Overlay 
  Plugin_CgProgramManager Plugin_ParticleFX
  RenderSystem_Direct3D11 RenderSystem_Direct3D9 RenderSystem_GL RenderSystem_GL3Plus RenderSystem_GLES RenderSystem_GLES2)
set(OGRE_RESET_VARS 
  OGRE_CONFIG_INCLUDE_DIR OGRE_INCLUDE_DIR 
  OGRE_FRAMEWORK_INCLUDES OGRE_FRAMEWORK_PATH OGRE_LIBRARY_FWK OGRE_LIBRARY_REL OGRE_LIBRARY_DBG
  OGRE_PLUGIN_DIR_DBG OGRE_PLUGIN_DIR_REL OGRE_MEDIA_DIR)
foreach (comp ${OGRE_COMPONENTS})
  set(OGRE_RESET_VARS ${OGRE_RESET_VARS}
    OGRE_${comp}_INCLUDE_DIR OGRE_${comp}_LIBRARY_FWK
    OGRE_${comp}_LIBRARY_DBG OGRE_${comp}_LIBRARY_REL
  )
endforeach (comp)
set(OGRE_PREFIX_WATCH ${OGRE_PREFIX_PATH} ${OGRE_PREFIX_SOURCE} ${OGRE_PREFIX_BUILD})
clear_if_changed(OGRE_PREFIX_WATCH ${OGRE_RESET_VARS})

if(NOT OGRE_STATIC)
	# try to locate Ogre via pkg-config
	use_pkgconfig(OGRE_PKGC "OGRE${OGRE_LIB_SUFFIX}")

	# Set the framework search path for OS X
    set(OGRE_FRAMEWORK_SEARCH_PATH
      ${CMAKE_FRAMEWORK_PATH}
      ~/Library/Frameworks
      /Library/Frameworks
      /System/Library/Frameworks
      /Network/Library/Frameworks
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/macosx/Release
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/macosx/Debug
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE}
    )
else()
	set(OGRE_LIBRARY_FWK "")
endif()

# locate Ogre include files
find_path(OGRE_CONFIG_INCLUDE_DIR NAMES OgreBuildSettings.h HINTS ${OGRE_INC_SEARCH_PATH} ${OGRE_FRAMEWORK_INCLUDES} ${OGRE_PKGC_INCLUDE_DIRS} PATH_SUFFIXES "OGRE")
find_path(OGRE_INCLUDE_DIR NAMES OgreRoot.h HINTS ${OGRE_CONFIG_INCLUDE_DIR} ${OGRE_INC_SEARCH_PATH} ${OGRE_FRAMEWORK_INCLUDES} ${OGRE_PKGC_INCLUDE_DIRS} PATH_SUFFIXES "OGRE")
set(OGRE_INCOMPATIBLE FALSE)

if (OGRE_INCLUDE_DIR)
  if (NOT OGRE_CONFIG_INCLUDE_DIR)
    set(OGRE_CONFIG_INCLUDE_DIR ${OGRE_INCLUDE_DIR})
  endif ()
  # determine Ogre version
  file(READ ${OGRE_INCLUDE_DIR}/OgrePrerequisites.h OGRE_TEMP_VERSION_CONTENT)
  get_preprocessor_entry(OGRE_TEMP_VERSION_CONTENT OGRE_VERSION_MAJOR OGRE_VERSION_MAJOR)
  get_preprocessor_entry(OGRE_TEMP_VERSION_CONTENT OGRE_VERSION_MINOR OGRE_VERSION_MINOR)
  get_preprocessor_entry(OGRE_TEMP_VERSION_CONTENT OGRE_VERSION_PATCH OGRE_VERSION_PATCH)
  get_preprocessor_entry(OGRE_TEMP_VERSION_CONTENT OGRE_VERSION_NAME OGRE_VERSION_NAME)
  set(OGRE_VERSION "${OGRE_VERSION_MAJOR}.${OGRE_VERSION_MINOR}.${OGRE_VERSION_PATCH}")
  pkg_message(OGRE "Found Ogre ${OGRE_VERSION_NAME} (${OGRE_VERSION})")

  # determine configuration settings
  set(OGRE_CONFIG_HEADERS
    ${OGRE_CONFIG_INCLUDE_DIR}/OgreBuildSettings.h
    ${OGRE_CONFIG_INCLUDE_DIR}/OgreConfig.h
  )
  foreach(CFG_FILE ${OGRE_CONFIG_HEADERS})
    if (EXISTS ${CFG_FILE})
      set(OGRE_CONFIG_HEADER ${CFG_FILE})
      break()
    endif()
  endforeach()
  if (OGRE_CONFIG_HEADER)
    file(READ ${OGRE_CONFIG_HEADER} OGRE_TEMP_CONFIG_CONTENT)
    has_preprocessor_entry(OGRE_TEMP_CONFIG_CONTENT OGRE_STATIC_LIB OGRE_CONFIG_STATIC)
    get_preprocessor_entry(OGRE_TEMP_CONFIG_CONTENT OGRE_THREAD_SUPPORT OGRE_CONFIG_THREADS)
    get_preprocessor_entry(OGRE_TEMP_CONFIG_CONTENT OGRE_THREAD_PROVIDER OGRE_CONFIG_THREAD_PROVIDER)
    get_preprocessor_entry(OGRE_TEMP_CONFIG_CONTENT OGRE_NO_FREEIMAGE OGRE_CONFIG_FREEIMAGE)
    if (OGRE_CONFIG_STATIC AND OGRE_STATIC)
    elseif (OGRE_CONFIG_STATIC OR OGRE_STATIC)
      pkg_message(OGRE "Build type (static, dynamic) does not match the requested one.")
      set(OGRE_INCOMPATIBLE TRUE)
    endif ()
  else ()
    pkg_message(OGRE "Could not determine Ogre build configuration.")
    set(OGRE_INCOMPATIBLE TRUE)
  endif ()
else ()
  set(OGRE_INCOMPATIBLE FALSE)
endif ()

find_library(OGRE_LIBRARY_REL NAMES ${OGRE_LIBRARY_NAMES} HINTS ${OGRE_LIB_SEARCH_PATH} ${OGRE_PKGC_LIBRARY_DIRS} ${OGRE_FRAMEWORK_SEARCH_PATH} PATH_SUFFIXES "" "Release" "RelWithDebInfo" "MinSizeRel")
find_library(OGRE_LIBRARY_DBG NAMES ${OGRE_LIBRARY_NAMES_DBG} HINTS ${OGRE_LIB_SEARCH_PATH} ${OGRE_PKGC_LIBRARY_DIRS} ${OGRE_FRAMEWORK_SEARCH_PATH} PATH_SUFFIXES "" "Debug")

make_library_set(OGRE_LIBRARY)

if (OGRE_INCOMPATIBLE)
  set(OGRE_LIBRARY "NOTFOUND")
endif ()

if("${OGRE_FRAMEWORK_INCLUDES}" STREQUAL NOTFOUND)
  unset(OGRE_FRAMEWORK_INCLUDES CACHE)
endif()
set(OGRE_INCLUDE_DIR ${OGRE_CONFIG_INCLUDE_DIR} ${OGRE_INCLUDE_DIR} ${OGRE_FRAMEWORK_INCLUDES})
list(REMOVE_DUPLICATES OGRE_INCLUDE_DIR)
findpkg_finish(OGRE)
add_parent_dir(OGRE_INCLUDE_DIRS OGRE_INCLUDE_DIR)
if (OGRE_SOURCE)
	# If working from source rather than SDK, add samples include
	set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} "${OGRE_SOURCE}/Samples/Common/include")
endif()

mark_as_advanced(OGRE_CONFIG_INCLUDE_DIR OGRE_MEDIA_DIR OGRE_PLUGIN_DIR_REL OGRE_PLUGIN_DIR_DBG)

if (NOT OGRE_FOUND)
  return()
endif ()


# look for required Ogre dependencies in case of static build and/or threading
if (OGRE_STATIC)
  set(OGRE_DEPS_FOUND TRUE)
  find_package(Cg QUIET)
  find_package(DirectX QUIET)
  find_package(FreeImage QUIET)
  find_package(Freetype QUIET)
  find_package(OpenGL QUIET)
  find_package(OpenGLES QUIET)
  find_package(OpenGLES2 QUIET)
  find_package(ZLIB QUIET)
  find_package(ZZip QUIET)
  if (UNIX AND NOT APPLE AND NOT ANDROID)
    find_package(X11 QUIET)
    find_library(XAW_LIBRARY NAMES Xaw Xaw7 PATHS ${DEP_LIB_SEARCH_DIR} ${X11_LIB_SEARCH_PATH})
    if (NOT XAW_LIBRARY OR NOT X11_Xt_FOUND)
      set(X11_FOUND FALSE)
    endif ()
  endif ()

  set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${ZZip_LIBRARIES} ${ZLIB_LIBRARIES} ${FreeImage_LIBRARIES} ${FREETYPE_LIBRARIES})

  if (APPLE AND NOT OGRE_BUILD_PLATFORM_APPLE_IOS AND NOT ANDROID)
    set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${X11_LIBRARIES} ${X11_Xt_LIBRARIES} ${XAW_LIBRARY} ${X11_Xrandr_LIB} ${Carbon_LIBRARIES} ${Cocoa_LIBRARIES})
  endif()
  
  if (NOT ZLIB_FOUND OR NOT ZZip_FOUND)
    set(OGRE_DEPS_FOUND FALSE)
  endif ()
  if (NOT FreeImage_FOUND AND NOT OGRE_CONFIG_FREEIMAGE)
    set(OGRE_DEPS_FOUND FALSE)
  endif ()
  if (NOT FREETYPE_FOUND)
    set(OGRE_DEPS_FOUND FALSE)
  endif ()
  if (UNIX AND NOT APPLE AND NOT ANDROID)
	if (NOT X11_FOUND)
      set(OGRE_DEPS_FOUND FALSE)
	endif ()
  endif ()
endif()
  if (OGRE_CONFIG_THREADS)
    if (OGRE_CONFIG_THREAD_PROVIDER EQUAL 1)
      if (OGRE_STATIC)
    	set(Boost_USE_STATIC_LIBS TRUE)
    	if(OGRE_BUILD_PLATFORM_APPLE_IOS)
          set(Boost_USE_MULTITHREADED OFF)
        endif()
      endif()
      
      set(OGRE_BOOST_COMPONENTS thread date_time)
      find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
      if(Boost_FOUND AND Boost_VERSION GREATER 104900)
        if(Boost_VERSION GREATER 105300)
            set(OGRE_BOOST_COMPONENTS thread date_time system atomic chrono)
        else()
            set(OGRE_BOOST_COMPONENTS thread date_time system chrono)
        endif()
      endif()

      find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
      if (NOT Boost_THREAD_FOUND)
        set(OGRE_DEPS_FOUND FALSE)
      else ()
        set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Boost_LIBRARIES})
        set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
      endif ()
    elseif (OGRE_CONFIG_THREAD_PROVIDER EQUAL 2)
      find_package(POCO QUIET)
      if (NOT POCO_FOUND)
        set(OGRE_DEPS_FOUND FALSE)
      else ()
        set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${POCO_LIBRARIES})
        set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} ${POCO_INCLUDE_DIRS})
      endif ()
    elseif (OGRE_CONFIG_THREAD_PROVIDER EQUAL 3)
      find_package(TBB QUIET)
      if (NOT TBB_FOUND)
        set(OGRE_DEPS_FOUND FALSE)
      else ()
        set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${TBB_LIBRARIES})
        set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS})
      endif ()
    endif ()
  endif ()
if (OGRE_STATIC)
  if (NOT OGRE_DEPS_FOUND)
    pkg_message(OGRE "Could not find all required dependencies for the OGRE-Next package.")
    set(OGRE_FOUND FALSE)
  endif ()
endif ()

if (NOT OGRE_FOUND)
  return()
endif ()


get_filename_component(OGRE_LIBRARY_DIR_REL "${OGRE_LIBRARY_REL}" PATH)
get_filename_component(OGRE_LIBRARY_DIR_DBG "${OGRE_LIBRARY_DBG}" PATH)
set(OGRE_LIBRARY_DIRS ${OGRE_LIBRARY_DIR_REL} ${OGRE_LIBRARY_DIR_DBG})

# find binaries
if (NOT OGRE_STATIC)
	if (WIN32)
		find_file(OGRE_BINARY_REL NAMES "OgreMain.dll" HINTS ${OGRE_BIN_SEARCH_PATH}
          PATH_SUFFIXES "" Release RelWithDebInfo MinSizeRel)
		find_file(OGRE_BINARY_DBG NAMES "OgreMain_d.dll" HINTS ${OGRE_BIN_SEARCH_PATH}
          PATH_SUFFIXES "" Debug )
	endif()
	mark_as_advanced(OGRE_BINARY_REL OGRE_BINARY_DBG)
endif()


#########################################################
# Find Ogre components
#########################################################

set(OGRE_COMPONENT_SEARCH_PATH_REL 
  ${OGRE_LIBRARY_DIR_REL}/..
  ${OGRE_LIBRARY_DIR_REL}/../..
  ${OGRE_BIN_SEARCH_PATH}
)
set(OGRE_COMPONENT_SEARCH_PATH_DBG
  ${OGRE_LIBRARY_DIR_DBG}/..
  ${OGRE_LIBRARY_DIR_DBG}/../..
  ${OGRE_BIN_SEARCH_PATH}
)

macro(ogre_find_component COMPONENT HEADER PATH_HINTS)
  set(OGRE_${COMPONENT}_FIND_QUIETLY ${OGRE_FIND_QUIETLY})
  findpkg_begin(OGRE_${COMPONENT})
  find_path(OGRE_${COMPONENT}_INCLUDE_DIR NAMES ${HEADER} HINTS ${OGRE_INCLUDE_DIRS} ${OGRE_PREFIX_SOURCE} PATH_SUFFIXES ${PATH_HINTS} ${COMPONENT} OGRE/${COMPONENT} )
  set(OGRE_${COMPONENT}_LIBRARY_NAMES "Ogre${COMPONENT}${OGRE_LIB_SUFFIX}")
  get_debug_names(OGRE_${COMPONENT}_LIBRARY_NAMES)
  find_library(OGRE_${COMPONENT}_LIBRARY_REL NAMES ${OGRE_${COMPONENT}_LIBRARY_NAMES} HINTS ${OGRE_LIBRARY_DIR_REL} ${OGRE_FRAMEWORK_PATH} PATH_SUFFIXES "" "Release" "RelWithDebInfo" "MinSizeRel")
  find_library(OGRE_${COMPONENT}_LIBRARY_DBG NAMES ${OGRE_${COMPONENT}_LIBRARY_NAMES_DBG} HINTS ${OGRE_LIBRARY_DIR_DBG} ${OGRE_FRAMEWORK_PATH} PATH_SUFFIXES "" "Debug")
  make_library_set(OGRE_${COMPONENT}_LIBRARY)
  findpkg_finish(OGRE_${COMPONENT})
  if (OGRE_${COMPONENT}_FOUND)
    # find binaries
    if (NOT OGRE_STATIC)
	  if (WIN32)
	    find_file(OGRE_${COMPONENT}_BINARY_REL NAMES "Ogre${COMPONENT}.dll" HINTS ${OGRE_COMPONENT_SEARCH_PATH_REL} PATH_SUFFIXES "" bin bin/Release bin/RelWithDebInfo bin/MinSizeRel Release)
	    find_file(OGRE_${COMPONENT}_BINARY_DBG NAMES "Ogre${COMPONENT}_d.dll" HINTS ${OGRE_COMPONENT_SEARCH_PATH_DBG} PATH_SUFFIXES "" bin bin/Debug Debug)
	  endif()
	  mark_as_advanced(OGRE_${COMPONENT}_BINARY_REL OGRE_${COMPONENT}_BINARY_DBG)
    endif()
  endif()
  unset(OGRE_${COMPONENT}_FIND_QUIETLY)
endmacro()

# look for Paging component
ogre_find_component(Paging OgrePaging.h "")
# look for Terrain component
ogre_find_component(Terrain OgreTerrain.h "")
# look for Property component
ogre_find_component(Property OgreProperty.h "")
# look for RTShaderSystem component
ogre_find_component(RTShaderSystem OgreRTShaderSystem.h "")
# look for Volume component
ogre_find_component(Volume OgreVolumePrerequisites.h "")
# look for Overlay component
ogre_find_component(Overlay OgreOverlaySystem.h "")
#look for HlmsPbs component
ogre_find_component(HlmsPbs OgreHlmsPbs.h Hlms/Pbs/)
#look for HlmsPbsMobile component
ogre_find_component(HlmsPbsMobile OgreHlmsPbsMobile.h Hlms/PbsMobile/)
#look for HlmsPbsMobile component
ogre_find_component(HlmsUnlit OgreHlmsUnlit.h Hlms/Unlit)
#look for HlmsUnlit component
ogre_find_component(HlmsUnlitMobile OgreHlmsUnlitMobile.h Hlms/UnlitMobile)

#########################################################
# Find Ogre plugins
#########################################################        
macro(ogre_find_plugin PLUGIN HEADER)
  # On Unix, the plugins might have no prefix
  if (CMAKE_FIND_LIBRARY_PREFIXES)
    set(TMP_CMAKE_LIB_PREFIX ${CMAKE_FIND_LIBRARY_PREFIXES})
    set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
  endif()
  
  # strip RenderSystem_ or Plugin_ prefix from plugin name
  string(REPLACE "RenderSystem_" "" PLUGIN_TEMP ${PLUGIN})
  string(REPLACE "Plugin_" "" PLUGIN_NAME ${PLUGIN_TEMP})
  
  # header files for plugins are not usually needed, but find them anyway if they are present
  set(OGRE_PLUGIN_PATH_SUFFIXES
    PlugIns PlugIns/${PLUGIN_NAME} Plugins Plugins/${PLUGIN_NAME} ${PLUGIN} 
    RenderSystems RenderSystems/${PLUGIN_NAME} ${ARGN})
  find_path(OGRE_${PLUGIN}_INCLUDE_DIR NAMES ${HEADER} 
    HINTS ${OGRE_INCLUDE_DIRS} ${OGRE_PREFIX_SOURCE}  
    PATH_SUFFIXES ${OGRE_PLUGIN_PATH_SUFFIXES})
  # find link libraries for plugins
  set(OGRE_${PLUGIN}_LIBRARY_NAMES "${PLUGIN}${OGRE_LIB_SUFFIX}")
  get_debug_names(OGRE_${PLUGIN}_LIBRARY_NAMES)
  set(OGRE_${PLUGIN}_LIBRARY_FWK ${OGRE_LIBRARY_FWK})
  find_library(OGRE_${PLUGIN}_LIBRARY_REL NAMES ${OGRE_${PLUGIN}_LIBRARY_NAMES}
    HINTS "${OGRE_BUILD}/lib" ${OGRE_LIBRARY_DIRS} ${OGRE_FRAMEWORK_PATH} PATH_SUFFIXES "" OGRE OGRE-${OGRE_VERSION} opt Release Release/opt RelWithDebInfo RelWithDebInfo/opt MinSizeRel MinSizeRel/opt)
  find_library(OGRE_${PLUGIN}_LIBRARY_DBG NAMES ${OGRE_${PLUGIN}_LIBRARY_NAMES_DBG}
    HINTS "${OGRE_BUILD}/lib" ${OGRE_LIBRARY_DIRS} ${OGRE_FRAMEWORK_PATH} PATH_SUFFIXES "" OGRE OGRE-${OGRE_VERSION} opt Debug Debug/opt)
  make_library_set(OGRE_${PLUGIN}_LIBRARY)

  if (OGRE_${PLUGIN}_LIBRARY OR OGRE_${PLUGIN}_INCLUDE_DIR)
    set(OGRE_${PLUGIN}_FOUND TRUE)
    if (OGRE_${PLUGIN}_INCLUDE_DIR)
      set(OGRE_${PLUGIN}_INCLUDE_DIRS ${OGRE_${PLUGIN}_INCLUDE_DIR})
    endif()
    set(OGRE_${PLUGIN}_LIBRARIES ${OGRE_${PLUGIN}_LIBRARY})
  endif ()

  mark_as_advanced(OGRE_${PLUGIN}_INCLUDE_DIR OGRE_${PLUGIN}_LIBRARY_REL OGRE_${PLUGIN}_LIBRARY_DBG OGRE_${PLUGIN}_LIBRARY_FWK)

  # look for plugin dirs
  if (OGRE_${PLUGIN}_FOUND)
    if (NOT OGRE_PLUGIN_DIR_REL OR NOT OGRE_PLUGIN_DIR_DBG)
      if (WIN32)
        set(OGRE_PLUGIN_SEARCH_PATH_REL 
          ${OGRE_LIBRARY_DIR_REL}/..
          ${OGRE_LIBRARY_DIR_REL}/../..
		  ${OGRE_BIN_SEARCH_PATH}
        )
        set(OGRE_PLUGIN_SEARCH_PATH_DBG
          ${OGRE_LIBRARY_DIR_DBG}/..
          ${OGRE_LIBRARY_DIR_DBG}/../..
		  ${OGRE_BIN_SEARCH_PATH}
        )
        find_path(OGRE_PLUGIN_DIR_REL NAMES "${PLUGIN}.dll" HINTS ${OGRE_PLUGIN_SEARCH_PATH_REL}
          PATH_SUFFIXES "" bin bin/Release bin/RelWithDebInfo bin/MinSizeRel Release)
        find_path(OGRE_PLUGIN_DIR_DBG NAMES "${PLUGIN}_d.dll" HINTS ${OGRE_PLUGIN_SEARCH_PATH_DBG}
          PATH_SUFFIXES "" bin bin/Debug Debug)
      elseif (UNIX)
        get_filename_component(OGRE_PLUGIN_DIR_TMP ${OGRE_${PLUGIN}_LIBRARY_REL} PATH)
        set(OGRE_PLUGIN_DIR_REL ${OGRE_PLUGIN_DIR_TMP} CACHE STRING "Ogre plugin dir (release)" FORCE)
	    get_filename_component(OGRE_PLUGIN_DIR_TMP ${OGRE_${PLUGIN}_LIBRARY_DBG} PATH)
        set(OGRE_PLUGIN_DIR_DBG ${OGRE_PLUGIN_DIR_TMP} CACHE STRING "Ogre plugin dir (debug)" FORCE)  
      endif ()
    endif ()
	
	# find binaries
	if (NOT OGRE_STATIC)
		if (WIN32)
			find_file(OGRE_${PLUGIN}_REL NAMES "${PLUGIN}.dll" HINTS ${OGRE_PLUGIN_DIR_REL})
			find_file(OGRE_${PLUGIN}_DBG NAMES "${PLUGIN}_d.dll" HINTS ${OGRE_PLUGIN_DIR_DBG})
		endif()
		mark_as_advanced(OGRE_${PLUGIN}_REL OGRE_${PLUGIN}_DBG)
	endif()
	
  endif ()

  if (TMP_CMAKE_LIB_PREFIX)
    set(CMAKE_FIND_LIBRARY_PREFIXES ${TMP_CMAKE_LIB_PREFIX})
  endif ()
endmacro(ogre_find_plugin)

ogre_find_plugin(Plugin_CgProgramManager OgreCgProgram.h PlugIns/CgProgramManager/include)
ogre_find_plugin(Plugin_ParticleFX OgreParticleFXPrerequisites.h PlugIns/ParticleFX/include)
ogre_find_plugin(RenderSystem_GL OgreGLRenderSystem.h RenderSystems/GL/include)
ogre_find_plugin(RenderSystem_GL3Plus OgreGL3PlusRenderSystem.h RenderSystems/GL3Plus/include)
ogre_find_plugin(RenderSystem_GLES OgreGLESRenderSystem.h RenderSystems/GLES/include)
ogre_find_plugin(RenderSystem_GLES2 OgreGLES2RenderSystem.h RenderSystems/GLES2/include)
ogre_find_plugin(RenderSystem_Direct3D9 OgreD3D9RenderSystem.h RenderSystems/Direct3D9/include)
ogre_find_plugin(RenderSystem_Direct3D11 OgreD3D11RenderSystem.h RenderSystems/Direct3D11/include)
        
if (OGRE_STATIC)
  # check if dependencies for plugins are met
  if (NOT DirectX_FOUND)
    set(OGRE_RenderSystem_Direct3D9_FOUND FALSE)
  endif ()
  if (NOT DirectX_D3D11_FOUND)
    set(OGRE_RenderSystem_Direct3D11_FOUND FALSE)
  endif ()
  if (NOT OPENGL_FOUND)
    set(OGRE_RenderSystem_GL_FOUND FALSE)
  endif ()
  if (NOT OPENGL_FOUND)
    set(OGRE_RenderSystem_GL3Plus_FOUND FALSE)
  endif ()
  if (NOT OPENGLES_FOUND)
    set(OGRE_RenderSystem_GLES_FOUND FALSE)
  endif ()
  if (NOT OPENGLES2_FOUND)
    set(OGRE_RenderSystem_GLES2_FOUND FALSE)
  endif ()
  if (NOT Cg_FOUND)
    set(OGRE_Plugin_CgProgramManager_FOUND FALSE)
  endif ()
  
  set(OGRE_RenderSystem_Direct3D9_LIBRARIES ${OGRE_RenderSystem_Direct3D9_LIBRARIES}
    ${DirectX_LIBRARIES}
  )

  set(OGRE_RenderSystem_Direct3D11_LIBRARIES ${OGRE_RenderSystem_Direct3D11_LIBRARIES}
    ${DirectX_D3D11_LIBRARIES}
  )
  set(OGRE_RenderSystem_GL_LIBRARIES ${OGRE_RenderSystem_GL_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
  set(OGRE_RenderSystem_GL3Plus_LIBRARIES ${OGRE_RenderSystem_GL3Plus_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
  set(OGRE_RenderSystem_GLES_LIBRARIES ${OGRE_RenderSystem_GLES_LIBRARIES}
    ${OPENGLES_LIBRARIES}
  )
  set(OGRE_RenderSystem_GLES2_LIBRARIES ${OGRE_RenderSystem_GLES2_LIBRARIES}
    ${OPENGLES2_LIBRARIES}
  )
  set(OGRE_Plugin_CgProgramManager_LIBRARIES ${OGRE_Plugin_CgProgramManager_LIBRARIES}
    ${Cg_LIBRARIES}
  )
endif ()

# look for the media directory
set(OGRE_MEDIA_SEARCH_PATH
  ${OGRE_SOURCE}
  ${OGRE_LIBRARY_DIR_REL}/..
  ${OGRE_LIBRARY_DIR_DBG}/..
  ${OGRE_LIBRARY_DIR_REL}/../..
  ${OGRE_LIBRARY_DIR_DBG}/../..
  ${OGRE_PREFIX_SOURCE}
)
set(OGRE_MEDIA_SEARCH_SUFFIX
  Samples/Media
  Media
  media
  share/OGRE/media
  share/OGRE/Media
)

clear_if_changed(OGRE_PREFIX_WATCH OGRE_MEDIA_DIR)
find_path(OGRE_MEDIA_DIR NAMES packs/cubemapsJS.zip HINTS ${OGRE_MEDIA_SEARCH_PATH}
  PATHS ${OGRE_PREFIX_PATH} PATH_SUFFIXES ${OGRE_MEDIA_SEARCH_SUFFIX})

