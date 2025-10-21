set(COMMON_SOURCES ${COMMON_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/boid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_trackball.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gl_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/moving_object.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/obstacle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fence.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/target.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ref_axis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/metrics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/simulation_context.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/waypoints.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/messages.cpp
    PARENT_SCOPE
)
