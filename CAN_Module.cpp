// ============================================
// Step 1: CAN Module Communication Only
// Raspberry Pi 4용 C++ 코드
// MCP2515 CAN 컨트롤러 통신
// ============================================

#include <iostream>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

using namespace std;

// ============================================
// 전역변수 선언
// ============================================
atomic<bool> running(true);
atomic<int32_t> Timer32(0);

// CAN Data
int32_t EncoderRead = 0;
int32_t EncoderDist = 0;
int32_t EncoderBase = 0;
int32_t InclinationX = 0;
int32_t InclinationY = 0;
bool DidEncoderBaseRead = false;

// Inclination Filter Buffers
int32_t InclinationX_BufX[3] = {0, 0, 0};
int32_t InclinationX_BufY = 0;
bool first_InclinationX = false;

int32_t InclinationY_BufX[3] = {0, 0, 0};
int32_t InclinationY_BufY = 0;
bool first_InclinationY = false;

// ============================================
// Union for Data Conversion
// ============================================
union Int32Data {
    uint8_t sbuf[4];
    int32_t lbuf;
};

// ============================================
// CAN Class
// ============================================
class CANInterface {
private:
    int can_socket;
    struct sockaddr_can addr;
    struct ifreq ifr;
    
public:
    CANInterface() : can_socket(-1) {}
    
    bool Init(const char* interface = "can0") {
        // CAN 소켓 생성
        can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket < 0) {
            cerr << "Error creating CAN socket" << endl;
            return false;
        }
        
        // 인터페이스 설정
        strcpy(ifr.ifr_name, interface);
        if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
            cerr << "Error: Interface " << interface << " not found" << endl;
            close(can_socket);
            return false;
        }
        
        // 주소 바인딩
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(can_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            cerr << "Error binding CAN socket" << endl;
            close(can_socket);
            return false;
        }
        
        cout << "CAN Interface " << interface << " initialized successfully" << endl;
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

// ============================================
// CAN Data Processing Functions
// ============================================
void SaveCANData(uint32_t can_id, uint8_t* data, uint8_t len) {
    Int32Data idata;
    
    if (can_id == 0x000002A0) {  // Encoder RM9000
        idata.sbuf[0] = data[0];
        idata.sbuf[1] = data[1];
        idata.sbuf[2] = data[2];
        idata.sbuf[3] = data[3];
        
        if (idata.lbuf >= 0x00800000) {
            EncoderRead = idata.lbuf - 0x01000000;
        } else {
            EncoderRead = idata.lbuf;
        }
        
        // 거리 값 계산
        EncoderDist = (EncoderRead * 1682) / 9999;
        
        if (!DidEncoderBaseRead) {
            EncoderBase = EncoderDist;
            DidEncoderBaseRead = true;
        }
        
        cout << "Encoder - Read: " << EncoderRead 
             << ", Dist: " << EncoderDist << endl;
    }
    else if (can_id == 0x0000028A || can_id == 0x0000028C) {  // Inclination
        // X축 경사
        idata.sbuf[0] = data[0];
        idata.sbuf[1] = data[1];
        idata.sbuf[2] = data[2];
        idata.sbuf[3] = data[3];
        
        if (idata.lbuf > 18000) idata.lbuf = idata.lbuf - 36000;
        if (idata.lbuf < -18000) idata.lbuf = idata.lbuf + 36000;
        
        if (!first_InclinationX) {
            InclinationX_BufX[0] = idata.lbuf;
            InclinationX_BufX[1] = idata.lbuf;
            InclinationX_BufX[2] = idata.lbuf;
            InclinationX_BufY = idata.lbuf;
            first_InclinationX = true;
        } else {
            InclinationX_BufX[2] = InclinationX_BufX[1];
            InclinationX_BufX[1] = InclinationX_BufX[0];
            InclinationX_BufX[0] = idata.lbuf;
            InclinationX_BufY = InclinationX;
        }
        InclinationX = (2*InclinationX_BufX[2] + 2*InclinationX_BufX[1] + 
                       4*InclinationX_BufX[0] + 8*InclinationX_BufY) / 16;
        
        // Y축 경사
        idata.sbuf[0] = data[4];
        idata.sbuf[1] = data[5];
        idata.sbuf[2] = data[6];
        idata.sbuf[3] = data[7];
        
        if (idata.lbuf > 18000) idata.lbuf = idata.lbuf - 36000;
        if (idata.lbuf < -18000) idata.lbuf = idata.lbuf + 36000;
        
        if (!first_InclinationY) {
            InclinationY_BufX[0] = idata.lbuf;
            InclinationY_BufX[1] = idata.lbuf;
            InclinationY_BufX[2] = idata.lbuf;
            InclinationY_BufY = idata.lbuf;
            first_InclinationY = true;
        } else {
            InclinationY_BufX[2] = InclinationY_BufX[1];
            InclinationY_BufX[1] = InclinationY_BufX[0];
            InclinationY_BufX[0] = idata.lbuf;
            InclinationY_BufY = InclinationY;
        }
        InclinationY = (2*InclinationY_BufX[2] + 2*InclinationY_BufX[1] + 
                       4*InclinationY_BufX[0] + 8*InclinationY_BufY) / 16;
        
        cout << "Inclination - X: " << InclinationX 
             << ", Y: " << InclinationY << endl;
    }
}

// ============================================
// CAN 수신 스레드
// ============================================
void CANReceiveTask(CANInterface& can) {
    struct can_frame frame;
    
    while (running) {
        if (can.ReadFrame(&frame)) {
            SaveCANData(frame.can_id, frame.data, frame.can_dlc);
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

// ============================================
// CAN Sync 전송 스레드 (50ms 주기)
// ============================================
void CANSyncTask(CANInterface& can) {
    uint8_t sync_data[1] = {0x00};
    
    while (running) {
        can.SendFrame(0x80, sync_data, 1);  // SYNC message
        this_thread::sleep_for(chrono::milliseconds(50));
    }
}

// ============================================
// 타이머 스레드 (1ms)
// ============================================
void TimerTask() {
    while (running) {
        Timer32++;
        this_thread::sleep_for(chrono::milliseconds(1));
    }
}

// ============================================
// Main
// ============================================
int main() {
    cout << "========================================" << endl;
    cout << "Step 1: CAN Communication Test" << endl;
    cout << "Raspberry Pi 4 - MCP2515" << endl;
    cout << "========================================" << endl;
    
    // CAN 인터페이스 초기화
    CANInterface can;
    if (!can.Init("can0")) {
        cerr << "Failed to initialize CAN interface" << endl;
        cerr << "Please setup CAN interface first:" << endl;
        cerr << "  sudo ip link set can0 type can bitrate 125000" << endl;
        cerr << "  sudo ip link set up can0" << endl;
        return -1;
    }
    
    // 스레드 시작
    thread timer_thread(TimerTask);
    thread can_rx_thread(CANReceiveTask, ref(can));
    thread can_sync_thread(CANSyncTask, ref(can));
    
    // 메인 루프 - 데이터 출력
    cout << "\nCAN communication started. Press Ctrl+C to exit.\n" << endl;
    
    int count = 0;
    while (running) {
        this_thread::sleep_for(chrono::seconds(1));
        
        cout << "\n--- Status Report (Timer: " << Timer32 << "ms) ---" << endl;
        cout << "Encoder Distance: " << EncoderDist << endl;
        cout << "Inclination X: " << InclinationX << ", Y: " << InclinationY << endl;
        
        count++;
        if (count >= 60) {  // 60초 테스트
            running = false;
        }
    }
    
    // 종료
    cout << "\nShutting down..." << endl;
    timer_thread.join();
    can_rx_thread.join();
    can_sync_thread.join();
    can.Close();
    
    cout << "Test completed successfully!" << endl;
    return 0;
}
