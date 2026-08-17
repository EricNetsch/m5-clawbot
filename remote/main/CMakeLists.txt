set(SOURCES
    esp_now/esp_now_init.c
    main.cpp
)

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS
                    "."
                    esp_now
                    )
