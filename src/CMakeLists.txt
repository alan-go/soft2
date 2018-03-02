add_library( SOFT SHARED
    system.cpp
    frame.cpp
    featurePointBox.cpp
    mapping.cpp
    tracking.cpp

    camera.cpp
    config.cpp
    g2o_types.cpp
)

target_link_libraries( SOFT
    ${THIRD_PARTY_LIBS}
)
