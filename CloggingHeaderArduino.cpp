#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include "json.hpp" // Подключение библиотеки nlohmann/json

using json = nlohmann::json;

std::mutex json_mutex;

// Функция для открытия и настройки COM-порта
int setupComPort(const std::string& portName) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        std::cerr << "Error opening COM port: " << portName << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting COM port attributes" << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                    // disable break processing
    tty.c_lflag = 0;                           // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                           // no remapping, no delays
    tty.c_cc[VMIN] = 1;                        // read doesn't block
    tty.c_cc[VTIME] = 1;                       // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);    // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);           // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);         // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting COM port attributes" << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

// Функция для чтения данных из COM-порта и обновления JSON
void listenToComPort(const std::string& portName, const std::string& jsonKey, const std::string& filePath) {
    int fd = setupComPort(portName);
    if (fd == -1) return;

    char buffer[4];
    while (true) {
        int bytesRead = read(fd, buffer, sizeof(buffer));
        if (bytesRead > 0) {
            int value = *reinterpret_cast<int*>(buffer);

            // Обновление JSON файла
            std::lock_guard<std::mutex> lock(json_mutex);

            std::ifstream inputFile(filePath);
            json jsonData;
            if (inputFile.is_open()) {
                inputFile >> jsonData; // Читаем текущие данные
                inputFile.close();
            }

            jsonData[jsonKey] = value; // Изменяем только нужное поле

            std::ofstream outputFile(filePath);
            if (outputFile.is_open()) {
                outputFile << jsonData.dump(4); // Перезаписываем файл с обновленными данными
                outputFile.close();
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Задержка 5 секунд
    }

    close(fd);
}

int main() {
    const std::string filePath = "data.json";

    // Запуск потоков для прослушивания COM-портов
    std::thread port1Thread(listenToComPort, "/dev/ttyUSB0", "CloggingHeader1", filePath);
    std::thread port2Thread(listenToComPort, "/dev/ttyUSB1", "CloggingHeader2", filePath);

    port1Thread.join();
    port2Thread.join();

    return 0;
}
