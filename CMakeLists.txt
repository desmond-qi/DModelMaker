cmake_minimum_required(VERSION 3.0)
project(ModelMaker)

set(ROOT_PATH .)

include_directories(
    ${ROOT_PATH}/include
)

link_directories(

)

aux_source_directory(${ROOT_PATH} SRC_TEST)

add_executable(ModelMakerTest
    ${SRC_TEST}
)

target_compile_definitions(ModelMakerTest PUBLIC
    _CRT_SECURE_NO_WARNINGS
    NDEBUG
    UNICODE
    CONSOLE
)

target_compile_options(ModelMakerTest PUBLIC
    /nologo
)

target_link_options(ModelMakerTest PUBLIC

)