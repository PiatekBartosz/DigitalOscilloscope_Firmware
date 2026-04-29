cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

set(APP_SRC_PATH "src/app")

target_sources(
    app
    PRIVATE
    src/main.c
    ${APP_SRC_PATH}/app.c
    ${APP_SRC_PATH}/afe_manager/afe_manager.c
    ${APP_SRC_PATH}/app_shell/app_shell.c
    ${APP_SRC_PATH}/parallel_link/parallel_link.c
    ${APP_SRC_PATH}/tcp_server/tcp_server.c
)

target_include_directories(
    app
    PRIVATE
    ${APP_SRC_PATH}
)
