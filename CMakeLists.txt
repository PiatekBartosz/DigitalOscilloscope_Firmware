cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

set(APP_SRC_PATH "src/app")

target_sources(
    app 
    PRIVATE 
    src/main.c
    ${APP_SRC_PATH}/app.c
    ${APP_SRC_PATH}/afe_manager/afe_manager.c
)

target_include_directories(
    app
    PRIVATE
    ${APP_SRC_PATH}
)
