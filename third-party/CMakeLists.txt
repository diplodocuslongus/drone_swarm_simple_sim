set(THIRD_PARTY_SOURCES ${THIRD_PARTY_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_demo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_impl_glut.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_impl_opengl2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_widgets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui.cpp
    PARENT_SCOPE
)
# ----------------------------------------------------
# Define the header-only library target for PLOG
# suggest by gemini
# ----------------------------------------------------
# Define the INTERFACE library with a simple name (plog)
add_library(plog INTERFACE)

# Define the INTERFACE_INCLUDE_DIRECTORIES using that name
target_include_directories(plog 
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/plog/include
)

# Create the namespaced ALIAS target that the application expects.
#    This allows to link using the modern 'plog::plog' syntax.
add_library(plog::plog ALIAS plog)
# IMGUI definition
add_library(imgui_lib INTERFACE)
# Add the directory containing the imgui headers to its interface
target_include_directories(imgui_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/imgui) 
