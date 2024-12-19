cmake_minimum_required(VERSION 3.10)

# Название проекта
project(CloggingHeaderArduino)

# Установка стандартов C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Добавление исполняемого файла
add_executable(CloggingHeaderArduino main.cpp)

# Подключение библиотеки nlohmann/json
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(CloggingHeaderArduino PRIVATE nlohmann_json::nlohmann_json)

# Дополнительные флаги компилятора
target_compile_options(CloggingHeaderArduino PRIVATE -Wall -Wextra -pedantic)
