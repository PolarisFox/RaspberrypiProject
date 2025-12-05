#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <cmath> 

// 리눅스 시스템 헤더 (SocketCAN)
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// Waveshare ADS1263 헤더 (SPI1 사용 전제)
#include "ADS1263.h" 

using namespace std;

// ---------------------------------------------------------
// 1. 데이터 처리용 공용체 및 변수
// ---------------------------------------------------------
union Int32Data {
    int32_t lbuf;       // 4바이트 정수 (Little Endian)
    uint8_t sbuf[4];    // 바이트 배열
};

// --- 출력할 최종 데이터 (메인 루프에서 사용) ---
double AdcVoltage = 0.0;   // ADC 전압
int32_t EncoderDist = 0;   // 거리 (계산됨)
int32_t InclinationX = 0;  // X축 각도 (필터링됨)
int32_t InclinationY = 0;  // Y축 각도 (필터링됨)

// --- 내부 계산용 변수 (CAN 파싱용) ---
int32_t EncoderRead = 0;

// 경사계 필터링 버퍼 (가중 평균용)
int32_t IncX_Buf[3] = {0};
int32_t IncX_Prev = 0;
bool first_IncX = false;

int32_t IncY_Buf[3] = {0};
int32_t IncY_Prev = 0;
bool first_IncY = false;

// 스레드 동기화용 뮤텍스
std::mutex data_mutex;
bool is_running = true;
int can_socket = -1;

// ---------------------------------------------------------
// 2. CAN 데이터 파싱 로직 (사용자 제공 로직 이식)
// ---------------------------------------------------------
void parseCANData(struct can_frame frame) {
    Int32Data idata;
    
    // 데이터 보호 (Mutex Lock)
    std::lock_guard<std::mutex> lock(data_mutex);

    // [엔코더 데이터 처리] (ID: 0x2A0)
    if (frame.can_id == 0x2A0) {
        idata.sbuf[0] = frame.data[0];
        idata.sbuf[1] = frame.data[1];
        idata.sbuf[2] = frame.data[2];
        idata.sbuf[3] = frame.data[3];

        // 24비트 음수/양수 보정
        if(idata.lbuf >= 0x00800000){
             EncoderRead = idata.lbuf - 0x01000000; 
        } else {
             EncoderRead = idata.lbuf;
        }

        // 거리 환산 공식 적용
        EncoderDist = (EncoderRead * 1682) / 9999;
    }
    
    // [경사계 데이터 처리] (ID: 0x28A or 0x28C)
    else if (frame.can_id == 0x28A || frame.can_id == 0x28C) {
        
        // --- X축 (앞 4바이트) ---
        idata.sbuf[0] = frame.data[0];
        idata.sbuf[1] = frame.data[1];
        idata.sbuf[2] = frame.data[2];
        idata.sbuf[3] = frame.data[3];

        // 180도 범위 정규화
        if(idata.lbuf > 18000) idata.lbuf -= 36000;
        if(idata.lbuf < -18000) idata.lbuf += 36000;

        // X축 가중 평균 필터 (Weighted Moving Average)
        if(!first_IncX) {
            IncX_Buf[0] = idata.lbuf; IncX_Buf[1] = idata.lbuf; IncX_Buf[2] = idata.lbuf;
            IncX_Prev = idata.lbuf;
            first_IncX = true;
        } else {
            IncX_Buf[2] = IncX_Buf[1];
            IncX_Buf[1] = IncX_Buf[0];
            IncX_Buf[0] = idata.lbuf;
            IncX_Prev = InclinationX;
        }
        InclinationX = (2*IncX_Buf[2] + 2*IncX_Buf[1] + 4*IncX_Buf[0] + 8*IncX_Prev) / 16;

        // --- Y축 (뒤 4바이트) ---
        idata.sbuf[0] = frame.data[4];
        idata.sbuf[1] = frame.data[5];
        idata.sbuf[2] = frame.data[6];
        idata.sbuf[3] = frame.data[7];

        if(idata.lbuf > 18000) idata.lbuf -= 36000;
        if(idata.lbuf < -18000) idata.lbuf += 36000;

        // Y축 가중 평균 필터
        if(!first_IncY) {
            IncY_Buf[0] = idata.lbuf; IncY_Buf[1] = idata.lbuf; IncY_Buf[2] = idata.lbuf;
            IncY_Prev = idata.lbuf;
            first_IncY = true;
        } else {
            IncY_Buf[2] = IncY_Buf[1];
            IncY_Buf[1] = IncY_Buf[0];
            IncY_Buf[0] = idata.lbuf;
            IncY_Prev = InclinationY;
        }
        InclinationY = (2*IncY_Buf[2] + 2*IncY_Buf[1] + 4*IncY_Buf[0] + 8*IncY_Prev) / 16;
    }
}

// ---------------------------------------------------------
// 3. CAN 수신 스레드
// ---------------------------------------------------------
void canReceiveThread(const char* ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("CAN Socket Error");
        return;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("CAN Bind Error");
        return;
    }

    struct can_frame frame;
    printf("[Thread] CAN Listener started on %s\n", ifname);

    while (is_running) {
        int nbytes = read(can_socket, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            parseCANData(frame); // 데이터 수신 즉시 파싱 및 저장
        }
    }
}

// ---------------------------------------------------------
// 4. CAN Sync 전송 스레드 (필요시 사용)
// ---------------------------------------------------------
void canSyncThread() {
    while(is_running) {
        if(can_socket >= 0) {
            struct can_frame sync_frame;
            sync_frame.can_id = 0x80; // [주의] 센서 매뉴얼에 맞는 ID로 수정 필요 (예: 0x80 or 0x01)
            sync_frame.can_dlc = 0;
            write(can_socket, &sync_frame, sizeof(sync_frame));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms 주기
    }
}

long long getCurrentTimeMillis() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

// ---------------------------------------------------------
// 5. 메인 함수 (ADC 읽기 + 데이터 통합 출력)
// ---------------------------------------------------------
int main() {
    // [초기화 1] 하드웨어 (GPIO, SPI)
    if (DEV_Module_Init() != 0) {
        printf("GPIO Init Failed. Run with sudo?\n");
        return -1;
    }

    // [초기화 2] ADC (SPI1 사용)
    printf("Initializing ADS1263 (SPI1)...\n");
    ADS1263_init_ADC1(ADS1263_400SPS); // 400Hz 속도
    ADS1263_SetMode(0);                // 0: Single-ended Mode

    // [초기화 3] CAN 스레드 시작
    printf("Starting CAN threads...\n");
    std::thread rxThread(canReceiveThread, "can0");
    std::thread txThread(canSyncThread); 

    printf("System Started. Data logging at 100ms...\n");

    const int SAMPLING_RATE_MS = 100; 

    while (true) {
        auto start_time = std::chrono::steady_clock::now();
        long long timestamp = getCurrentTimeMillis();

        // --- [STEP 1] ADC 데이터 읽기 ---
        // 채널 0번 (AIN0) 읽기
        UDOUBLE raw_adc = ADS1263_GetChannalValue(0); 
        
        // 전압 변환 (ADS1263 32bit -> Voltage)
        if((raw_adc >> 31) == 1) // 음수 처리
            AdcVoltage = -(double)(0xFFFFFFFF - raw_adc + 1) * 2.5 / 0x80000000;
        else
            AdcVoltage = (double)raw_adc * 2.5 / 0x7FFFFFFF;

        // --- [STEP 2] CAN 데이터 스냅샷 가져오기 ---
        int32_t curr_dist = 0, curr_incX = 0, curr_incY = 0;
        {
            // 뮤텍스로 보호된 최신 데이터를 복사해옴
            std::lock_guard<std::mutex> lock(data_mutex);
            curr_dist = EncoderDist;
            curr_incX = InclinationX;
            curr_incY = InclinationY;
        }

        // --- [STEP 3] 통합 데이터 출력 ---
        // [시간] ADC전압 | 거리 | X각도 | Y각도
        std::cout << "[T:" << timestamp << "] "
                  << "ADC: " << std::fixed << std::setprecision(4) << AdcVoltage << "V | "
                  << "Dist: " << curr_dist << " | "
                  << "IncX: " << std::fixed << std::setprecision(2) << (double)curr_incX/100.0 << " | "
                  << "IncY: " << std::fixed << std::setprecision(2) << (double)curr_incY/100.0 
                  << std::endl;

        // --- [STEP 4] 100ms 주기 대기 ---
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        if (elapsed < SAMPLING_RATE_MS) {
            std::this_thread::sleep_for(std::chrono::milliseconds(SAMPLING_RATE_MS - elapsed));
        }
    }

    is_running = false;
    rxThread.join();
    txThread.join();
    DEV_Module_Exit();

    return 0;
}