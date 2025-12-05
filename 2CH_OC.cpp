#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cmath>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>

using namespace std;

atomic<bool> running(true);
mutex data_mutex;

// CAN 수신 및 송신 메시지 카운트
atomic<int> can0_rx_count(0);
atomic<int> can1_rx_count(0);
atomic<int> can0_tx_count(0);
atomic<int> can1_tx_count(0);

class CANInterface {
private:
    int can_socket = -1;
    string iface;
public:
    CANInterface(const string& interface_name) : iface(interface_name) {}

    bool Init() {
        can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket < 0) {
            cerr << "Failed to create CAN socket for " << iface << endl;
            return false;
        }
        struct ifreq ifr;
        strcpy(ifr.ifr_name, iface.c_str());
        if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
            cerr << "Failed to get interface index for " << iface << endl;
            close(can_socket);
            return false;
        }
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            cerr << "Failed to bind CAN socket for " << iface << endl;
            close(can_socket);
            return false;
        }
        cout << iface << " initialized successfully." << endl;
        return true;
    }

    bool ReadFrame(struct can_frame* frame) {
        int nbytes = read(can_socket, frame, sizeof(struct can_frame));
        return (nbytes == sizeof(struct can_frame));
    }

    bool SendFrame(uint32_t id, uint8_t* data, uint8_t len) {
        struct can_frame frame;
        frame.can_id = id;
        frame.can_dlc = len;
        memcpy(frame.data, data, len);
        int nbytes = write(can_socket, &frame, sizeof(struct can_frame));
        return (nbytes == sizeof(struct can_frame));
    }

    void Close() {
        if (can_socket >= 0) {
            close(can_socket);
            can_socket = -1;
        }
    }

    ~CANInterface() {
        Close();
    }
};

void ProcessCANData(int ch, uint32_t can_id, uint8_t* data, uint8_t dlc) {
    // 예시 처리 - 데이터 출력
    lock_guard<mutex> lock(data_mutex);
    cout << "[CH" << ch << "] Received CAN ID: 0x" << hex << can_id << dec
         << " DLC: " << (int)dlc << " Data: ";
    for (int i = 0; i < dlc; i++) {
        printf("%02X ", data[i]);
    }
    cout << endl;
}

void CANReceiveTask(CANInterface& can, int ch, atomic<int>& rx_count) {
    struct can_frame frame;
    while (running) {
        if (can.ReadFrame(&frame)) {
            ProcessCANData(ch, frame.can_id, frame.data, frame.can_dlc);
            rx_count++;
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

void CANSendTask(CANInterface& can, int ch, atomic<int>& tx_count) {
    int counter = 0;
    while (running) {
        uint8_t data[8] = { (uint8_t)(counter & 0xFF), (uint8_t)((counter >> 8) & 0xFF), 0,0,0,0,0,0 };
        uint32_t id = 0x100 + ch; // 채널별 ID 차별화
        if (can.SendFrame(id, data, 8)) {
            tx_count++;
            cout << "[CH" << ch << "] Sent CAN ID: 0x" << hex << id << dec << endl;
        } else {
            cerr << "[CH" << ch << "] Failed to send CAN frame" << endl;
        }
        counter++;
        this_thread::sleep_for(chrono::milliseconds(333)); // 초당 약 3회 송신
    }
}

void CPULoadTask() {
    cout << "CPU Load Task started." << endl;
    while (running) {
        volatile double res = 0.0;
        for (int i = 0; i < 100000; i++) {
            res += sqrt(sin(i) * cos(i));
        }
    }
}

void MonitorTask() {
    while (running) {
        this_thread::sleep_for(chrono::seconds(1));
        int rx0 = can0_rx_count.exchange(0);
        int rx1 = can1_rx_count.exchange(0);
        int tx0 = can0_tx_count.exchange(0);
        int tx1 = can1_tx_count.exchange(0);

        cout << "=== Performance Monitor ===" << endl;
        cout << "CAN0 RX count last 1 sec: " << rx0 << endl;
        cout << "CAN1 RX count last 1 sec: " << rx1 << endl;
        cout << "CAN0 TX count last 1 sec: " << tx0 << endl;
        cout << "CAN1 TX count last 1 sec: " << tx1 << endl;
        cout << "===========================" << endl;
    }
}

int main() {
    cout << "=== 2-CH CAN Communication with CPU Load Test ===" << endl;

    CANInterface can0("can0");
    CANInterface can1("can1");

    if (!can0.Init() || !can1.Init()) {
        cerr << "CAN Interface initialization failed." << endl;
        return -1;
    }

    thread can0_rx_thread(CANReceiveTask, ref(can0), 0, ref(can0_rx_count));
    thread can1_rx_thread(CANReceiveTask, ref(can1), 1, ref(can1_rx_count));
    thread can0_tx_thread(CANSendTask, ref(can0), 0, ref(can0_tx_count));
    thread can1_tx_thread(CANSendTask, ref(can1), 1, ref(can1_tx_count));
    thread cpu_load_thread(CPULoadTask);
    thread monitor_thread(MonitorTask);

    this_thread::sleep_for(chrono::seconds(60));
    running = false;

    can0_rx_thread.join();
    can1_rx_thread.join();
    can0_tx_thread.join();
    can1_tx_thread.join();
    cpu_load_thread.join();
    monitor_thread.join();

    cout << "Test finished." << endl;
    return 0;
}
