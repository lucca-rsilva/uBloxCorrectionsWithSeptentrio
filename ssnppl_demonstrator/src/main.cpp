#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <array>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "utils.hpp"
#include <PPL_PublicInterface.h>

// =============================================================
// Thread-safe queue template
// =============================================================
template <typename T>
class SafeQueue {
private:
    std::queue<T> q;
    std::mutex m;
    std::condition_variable cv;
public:
    void push(const T& value) {
        {
            std::lock_guard<std::mutex> lock(m);
            q.push(value);
        }
        cv.notify_one();
    }

    bool try_pop(T& out) {
        std::unique_lock<std::mutex> lock(m);
        if (q.empty()) return false;
        out = std::move(q.front());
        q.pop();
        return true;
    }

    void wait_and_pop(T& out) {
        std::unique_lock<std::mutex> lock(m);
        cv.wait(lock, [&]{ return !q.empty(); });
        out = std::move(q.front());
        q.pop();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(m);
        return q.empty();
    }
};

// =============================================================
// Globals
// =============================================================
SafeQueue<std::vector<uint8_t>> queue_septentrio;
SafeQueue<std::vector<uint8_t>> queue_spartn;
SafeQueue<std::vector<uint8_t>> queue_rtcm;

std::atomic<bool> running(true);
// std::mutex ppl_mutex; // Protects all PPL_* calls

// =============================================================
// Serial setup helper
// =============================================================
int open_serial(const char* device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(device);
        return -1;
    }

    struct termios tty{};
    tcgetattr(fd, &tty);
    cfmakeraw(&tty);

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

// =============================================================
// Thread functions
// =============================================================
void thread_read_serial(const char* device, int baudrate, SafeQueue<std::vector<uint8_t>>* queue) {
    int fd = open_serial(device, baudrate);
    if (fd < 0) return;

    uint8_t buf[8092];
    while (running) {
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            std::vector<uint8_t> data(buf, buf + n);
            queue->push(std::move(data));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    close(fd);
}

void thread_write_rtcm() {
    // Open the same serial port as Septentrio for writing
    int fd = open_serial("/dev/ttyACM0", B115200);
    if (fd < 0) {
        std::cerr << "Failed to open /dev/ttyACM0 for RTCM output\n";
        return;
    }

    while (running) {
        std::vector<uint8_t> rtcm;
        queue_rtcm.wait_and_pop(rtcm);  // blocks until new RTCM data is available

        if (!rtcm.empty()) {
            ssize_t written = write(fd, rtcm.data(), rtcm.size());
            if (written < 0) {
                perror("Failed to write RTCM to serial");
            } else {
                std::vector<int> rtcm_id = identifyRTCM3MessageIDs(rtcm.data(), rtcm.size());
                std::cout << "Sending RTCM3 messages";
                if (rtcm_id.size() > 0)
                {
                    std::cout << ", id = ";
                    for (int &id : rtcm_id)
                        std::cout << id << " ";
                }

                std::cout << std::endl;

                std::cout << "RTCM out: " << written << " bytes\n";
            }
        }
    }

    close(fd);
}

// =============================================================
// Main processing loop
// =============================================================
int main() {
    std::cout << "Initializing Point Perfect Library..." << std::endl;
    ePPL_ReturnStatus ePPLRet = PPL_Initialize(PPL_CFG_ENABLE_IP_CHANNEL);
    if (ePPLRet != ePPL_Success) {
        std::cerr << "FAILED to initialize PPL" << std::endl;
        return 1;
    }

    std::thread t_sept(thread_read_serial, "/dev/ttyACM0", B115200, &queue_septentrio);
    std::thread t_spartn(thread_read_serial, "/dev/ttyUSB0", B9600, &queue_spartn);
    std::thread t_rtcm_writer(thread_write_rtcm); // uses the same serial as t_sept

    while (running) {
        std::vector<uint8_t> msg;

        // Process Septentrio input
        if (queue_septentrio.try_pop(msg)) {
            // std::lock_guard<std::mutex> lock(ppl_mutex);
            ePPL_ReturnStatus ret = PPL_SendRcvrData(msg.data(), msg.size());
            if (ret != ePPL_Success)
                std::cout << "FAILED TO SEND RCVR DATA" << std::endl;
            else
                std::cout << "Ephemeris Received. Size:" << msg.size() << std::endl;
        }

        // Process SPARTN input
        if (queue_spartn.try_pop(msg)) {
            // std::lock_guard<std::mutex> lock(ppl_mutex);
            ePPL_ReturnStatus ret = PPL_SendSpartn(msg.data(), msg.size());
            if (ret != ePPL_Success) {
                std::cout << "FAILED TO SEND SPARTN SERIAL INPUT DATA: " << ret << std::endl;
            } else {
                std::cout << "\nNew SPARTN Serial Message received. Message Size: " << msg.size() << std::endl;
                std::array<uint8_t, PPL_MAX_RTCM_BUFFER> rtcm_buf;
                uint32_t rtcm_size = 0;
                PPL_GetRTCMOutput(rtcm_buf.data(), PPL_MAX_RTCM_BUFFER, &rtcm_size);

                if (rtcm_size > 0) {
                    std::cout << "\nNew RTCM generated. Size: " << rtcm_size << std::endl;
                    queue_rtcm.push(std::vector<uint8_t>(
                        rtcm_buf.begin(), rtcm_buf.begin() + rtcm_size));
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    t_sept.join();
    t_spartn.join();
    t_rtcm_writer.join();
    return 0;
}
