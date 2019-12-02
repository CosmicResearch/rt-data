cmake_minimum_required(VERSION 3.1)
project(proba-xsens)

SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage") # Flags para debugar
SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)
set(CMAKE_CXX_STANDARD 17)

file(GLOB SRC_FILES Application.cpp) # Buscamos los archivos con codigo

add_executable(proba_xsens ${SRC_FILES}) # Creamos el ejecutable

find_package(Threads REQUIRED) # Añadimos dependencias
find_package(g3logger)
find_package(nlohmann_json)
#find_package(SQLiteCpp)
#set(CURL_LIBRARY "-lcurl")
#find_package(CURL REQUIRED)
target_link_libraries( # Linkamos las librerias necesarias
    proba_xsens
    rtdata
    Threads::Threads
    nlohmann_json::nlohmann_json
    SQLiteCpp
    sqlite3
    dl
    g3logger
    #curl
)

add_definitions(-DWITH_G3LOG) # Activamos el logging
