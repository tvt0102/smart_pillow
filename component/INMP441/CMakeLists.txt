set(app_src inmp441.c)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES driver log)