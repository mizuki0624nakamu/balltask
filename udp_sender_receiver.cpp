
#include <iostream>
#include <future>
#include <thread>

#include "dataStruct.h"
#include "crl_libs/UDP/UDPClient.h"
#include "crl_libs/UDP/UDPServer.h"
#include<math.h>
#include <conio.h>
#pragma comment(lib, "ws2_32.lib")
#include "unity.h"
#include "LPF.h"
//#include<math.h>
#include <ctime>
#include <chrono>

#ifdef _WINDOWS
#define SLEEP(time) Sleep(time)
#else
#define SLEEP(time) usleep(time * 1000)
#endif
//#ifndef UDP_PC_MBED_UNITY_H
#define UDP_PC_MBED_UNITY_H


// ���M�p
//------------------------------------------------------------
#define SERVER_IP "127.0.0.1"  // UnityIP address
#define SERVER_PORT 1234  // Unity PORT no
#define ARRAY_SIZE 11  //
#define INTERVAL_MS 17  // send msec
// ��M�p
//------------------------------------------------------------
#define PORT 1234  //
const double Pi = 3.1415926535897932384626433832795028841971;
const double um = 55.0;
const double ug = 9.0;
const double goal[30] = {20.0, 16.5, 21.5, 15.0, 8.0, 22.0, 14.0, 23.0, 18.5, 12.0, 20.0, 16.5, 21.5, 15.0, 8.0, 22.0, 14.0, 23.0, 18.5, 12.0,20.0, 16.5, 21.5, 15.0, 8.0, 22.0, 14.0, 23.0, 18.5, 12.0};
const double next_goal = 23.0;
double uu = PI/3.0;
const double h = 1.6;
double ki = 3;
int fc = 10;
int sw;

double T = 1 / (2 * Pi * fc);


/*!
 * @brief UDP Receiver
 * @param[in] ip IP address
 * @param[in] port Port number
 * @param[in] data Data to be sent
 * @param[in] size Size of data
 * @return Received size
 */
double GetTime() {
    /* ����N�����̂ݏ������Ԃ��L�^���� */
    static bool first_flag = true;  //����N���t���O
    static std::chrono::system_clock::time_point start, now;
    if (first_flag) {
        start = std::chrono::system_clock::now();
        first_flag = false;
    }
    now = std::chrono::system_clock::now(); //���݂̎��Ԃ��擾

    return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - start).count() * 0.001 *
                               0.001 * 0.001);  //�o�ߎ��Ԃ��v�Z
}

void receive_func() {
    UDPClient udp_client("", 15968);
    toPC topc;
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cout << "Winsock faild" << std::endl;
        return;
    }
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cout << "socket create faild" << std::endl;
        WSACleanup();
        return;
    }
    sockaddr_in serverAddr;

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);

    int received_size;


    double nowtime = 0.0;
    double reftime = 0.0;
    double Ts = 0.01;


    std::cout << "Start UDP Receiving!!" << std::endl;
    int calc_flag = 0;
    int data_count, vn;
    int task_initialize = 0;
    int i,i2, hc;
    int j, actual_data_count, z, k;
    int stop;

    double time = 0.0;
    double v, hv,bv,ba;
    double pt, first_x, pv, lt, lx;
    double x;
    double x_f;
    double v_all;
    double arrx[3000];
    double arrt[3000];
    double arrv[3000];
    double arra[3000];
    double lpv[3000];
    double lpa[3000];
    double lpx[3000];
    double ux, uy, ut=0;
    int count = 0;
    double base;
    int base_count = 0;
    int disp;
    double diff[31];
    double value[31];
    double speed[31];

    for (i2 = 0; i2 < 30; i2++) {
        diff[i2] = 0;
        value[i2] = 0;
        speed[i2] = 0;

    }

//    double usend[5000][3];




    while (1) {
        nowtime = GetTime();
//      if(Ts < nowtime-reftime) {
//        std::cout << "test" << std::endl;
        received_size = udp_client.receive(reinterpret_cast<char *>(&topc), sizeof(toPC));
//        std::cout << topc.tim1_pulse << std::endl;
        if(count == 0 && base_count == 0){
            base_count = 1;
            x = topc.tim1_pulse;
            x = x / 1000000.0000000;
            base = x;
        }

//    std::cout << "Received size: " << received_size << std::endl;
//    std::cout << "Received size: " << received_size << " data: " << topc.tim1_pulse << std::endl;





        if (topc.switch_state) {
            x = topc.tim1_pulse;
            x = x / 1000000.0000000;
//            std::cout << x << std::endl;


            if (task_initialize == 0) {
//                  time=0.0;
                x_f = x;
                data_count = 0;
                x_f = x_f + 0.01;
                for (i = 0; i < 3000; i++) {
                    arrx[i] = 0;
                    arrt[i] = 0;
                    arra[i] = 0;
                    arrv[i] = 0;
                    lpa[i] = 0;
                    lpx[i] = 0;
                    lpv[i] = 0;

                }
            } else if (x_f < x) {
                calc_flag = 1;
//                  time = nowtime;
                arrx[data_count] = x;
                arrt[data_count] = time;

//                  std::cout << nowtime << "x1 = " << x << " x2 = " << topc.switch_state << " x3 = " << topc.tim1_pulse<< " x4 = " << topc.tim8_pulse << std::endl;
//                  std::cout << arrt[data_count] << "," << arrx[data_count] << std::endl;

//                  time = nowtime;
                data_count++;
            }


            task_initialize++;

        } else if (calc_flag == 1) { // Calclation only once
            std::cout << "off" << std::endl;
            //UDP

            calc_flag = 0;
            v_all = 0;
            stop = 0;
            yp = h;
            xp = 0;
            vn = 0;
            // Do all of data
            for (j = 0; j < data_count; j++) {
//                if (j == 0) {
//                    pt = arrt[j];
//                    first_x = arrx[j];
//                } else
                if (j == 1) {
                    pv = arrx[j] - arrx[j - 1];
                    arrv[j - 1] = pv;

                } else {
                    arrv[j - 1] = arrx[j] - arrx[j - 1];
                    arra[j - 2] = arrv[j - 1] - arrv[j - 2];
                    if (arra[j - 2] <= 0) {
                        lt = arrt[j - 1];
                        lx = arrx[j - 1];
                    }
                    if (j == data_count - 1) {
                        actual_data_count = j;
                    }




//                      pt = arrt[j];
//                      first_x = arrx[j];
//                      actual_data_count = j;
                }

            }
            LPF lpf_x(20.0, 0.001);
            LPF lpf_v(5.0, 0.001);
            LPF lpf_a(5.0, 0.001);
            for (k = 0; k <= actual_data_count; k++) {

                if (k == 0) {
                    hc = 0;
                    lpv[k] = 0;
                    lpa[k] = 0;
                    lpx[k] = lpf_x.Get(arrx[k]);
                } else if (k == 1) {
                    lpa[k] = 0;
//                      lpv[k] = lpv[k-1] + ((arrt[k]-arrt[k-1])/T)*(arrv[k]-arrv[k-1]);
//                    lpv[k] = lpf_v.Get(arrv[k - 1]);
                    lpx[k] = lpf_x.Get(arrx[k]);
                    bv = (arrx[k]-arrx[k-1]) /0.001;
                    lpv[k] = lpf_v.Get(bv);

                } else {
                    lpv[k] = lpf_v.Get(arrv[k - 1]);
                    lpa[k] = lpf_a.Get(arra[k - 2]);
                    lpx[k] = lpf_x.Get(arrx[k]);
                    bv = (arrx[k]-arrx[k-1]) /0.001;
                    lpv[k] = lpf_v.Get(bv);
                    ba = (arrv[k]-arrv[k-1]) /0.001;
                    lpa[k] = lpf_a.Get(ba);

                    if (k > 30 && lpa[k] < 0 && hc == 0) {
                        hv = lpv[k];
                        hv = hv*8 ;
                        hc = 1;
                    }
//                      lpv[k] = lpv[k-1] + ((arrt[k]-arrt[k-1])/T)*(arrv[k]-lpv[k-1]);
//                      lpa[k] = lpa[k-1] + ((arrt[k]-arrt[k-1])/T)*(arra[k]-lpa[k-1]);

                }

            }
//            FILE *file;

//            file = fopen("tes1.csv", "w");
//            double csv_time = 0.0;
//            for (z = 0; z < actual_data_count; z++) {
//
//                fprintf(file, "%f,%f,%f,%f\n", csv_time, arrx[z], lpv[z], lpa[z]);
//                csv_time += 0.001;
//            }
//            fclose(file);
//              hv = (lx-first_x) / (lt-pt);
            std::cout << hv << "," << ki << std::endl;
            task_initialize = 0;


//            std::thread t1(SEND, hv, ki, h,goal,goal);
//            t1.join();
            uy = h;
            ut = 0;
            double data[ARRAY_SIZE] = { 0.0, 0.0, 0.0,0.0,0.0, 0.0,0.0,0.0 ,0.0,0.0,0.0};
            while(uy>=0){
                ux = (um*hv/ki)*(1-exp(-(ki)*ut/um))*cos(uu);
                uy = (um/ki)*((um*ug/ki)+hv*cos(uu))*(1-exp(-(ki)*ut/um))-um*ug*ut/ki+h;
                ut = ut+0.017;
                data[0] = ux;
                data[1] = uy + 0.6;
                data[2] = 0;
                data[3] = goal[count];
                data[4] = 0.51;
                data[5] = 0.0;
                data[6] = -8;
                data[7] = 0.52;
                data[8] = 0.0;
                data[9] = 0;
                disp = 0;
                data[10] = 0.0;
                if (uy < 0.0) {
                    data[6] = ux;
                }
                std::string message =
                        std::to_string(data[0]) + "," + std::to_string(data[1]) + "," + std::to_string(data[2]) + "," +
                        std::to_string(data[3]) + "," + std::to_string(data[4]) + "," + std::to_string(data[5]) + "," +
                        std::to_string(data[6]) + "," + std::to_string(data[7]) + "," + std::to_string(data[8])+ "," +
                        std::to_string(disp) + "," + std::to_string(data[10]);
                std::cout << message << std::endl;
                int result = sendto(sock, message.c_str(), message.length(), 0, (sockaddr *) &serverAddr, sizeof(serverAddr));
                if (result == SOCKET_ERROR) {
                    std::cout << "date send faild" << std::endl;
                }
                Sleep(17);
            }
            data[6] = ux;
            std::string message1 =
                    std::to_string(data[0]) + "," + std::to_string(data[1]) + "," + std::to_string(data[2]) + "," +
                    std::to_string(data[3]) + "," + std::to_string(data[4]) + "," + std::to_string(data[5]) + "," +
                    std::to_string(data[6]) + "," + std::to_string(data[7]) + "," + std::to_string(data[8]) + "," +
                    std::to_string(disp) + "," + std::to_string(data[10]);
            std::cout << message1 << std::endl;
            int result1 = sendto(sock, message1.c_str(), message1.length(), 0, (sockaddr *) &serverAddr, sizeof(serverAddr));
            Sleep(5000);
//            x = topc.tim1_pulse;
//            x = x / 1000000.0000000;
//            data[0] = -5;
//            data[3] = goal[count+1];
//            data[6] = -8;
//
//            std::string message2 =
//                    std::to_string(data[0]) + "," + std::to_string(data[1]) + "," + std::to_string(data[2]) + "," +
//                    std::to_string(data[3]) + "," + std::to_string(data[4]) + "," + std::to_string(data[5]) + "," +
//                    std::to_string(data[6]) + "," + std::to_string(data[7]) + "," + std::to_string(data[8]) + "," +
//                    std::to_string(data[9]) + "," + std::to_string(data[10]);
//            std::cout << message2 << std::endl;
//            int result2 = sendto(sock, message2.c_str(), message2.length(), 0, (sockaddr *) &serverAddr, sizeof(serverAddr));





            std::cout << ux << std::endl;
            std::cout << uy << std::endl;
            if(count == 0){
                if(ux < goal[0]){
                    sw=1;
                }else{
                    sw=2;
                }
            }
            if(count > 10) {
                if (sw == 1) {
                    if(count < 20) {
                        ki = ki - 0.2;
                    }else{
                        ki = ki + 0.2;
                    }
                } else {
                    if(count < 20) {
                        ki = ki + 0.2;
                    }else{
                        ki = ki - 0.2;
                    }
                }
            }
            diff[count] = goal[count] - ux;
            value[count] = ux;
            speed[count] = hv;

            std::cout << count << std::endl;
            std::cout << diff[count] << value[count] << speed[count]  << std::endl;
            count += 1;

            if(count == 31) {
                FILE *file;

                file = fopen("tes1.csv", "w");
                for (z = 0; z < 30; z++) {
                    fprintf(file, "%d,%f,%f,%f\n", z, diff[z], value[z], speed[z]);
                }


//                for (z = 0; z <= actual_data_count; z++) {
//                    fprintf(file, "%f,%f,%f,%f,%f,%f,%f\n", arrt[z], lpx[z], lpv[z], lpa[z], arrx[z], arrv[z], arra[z]);
//                }

                fclose(file);
            }

        }else{
            double datau[ARRAY_SIZE] ;
            x = topc.tim1_pulse;
            x = x / 1000000.0000000;
            datau[0] = -5.0;
            datau[1] = 0.0;
            datau[2] = 0.0;
            datau[3] = goal[count];
            datau[4] = 0.51;
            datau[5] = 0.0;
            datau[6] = -8.0;
            datau[7] = 0.0;
            datau[8] = 0.0;
            datau[9] = 1;
            disp = 1;
            datau[10] = x - base;

            std::string message3 =
                    std::to_string(datau[0]) + "," + std::to_string(datau[1]) + "," + std::to_string(datau[2]) + "," +
                    std::to_string(datau[3]) + "," + std::to_string(datau[4]) + "," + std::to_string(datau[5]) + "," +
                    std::to_string(datau[6]) + "," + std::to_string(datau[7]) + "," + std::to_string(datau[8]) + "," +
                    std::to_string(disp) + "," + std::to_string(datau[10]);

            int result3 = sendto(sock, message3.c_str(), message3.length(), 0, (sockaddr *) &serverAddr, sizeof(serverAddr));
        }
        reftime = nowtime;
        time = time + 0.001;
//      }
    }
}

// main function
int main() {
    GetTime();
    UDPServer udp_server("192.168.2.64", 12345);
    toMbed to_mbed;
    int sent_size;

    auto th1 = std::async(std::launch::async, receive_func); // start receiving thread

    std::cout << "Start UDP Sending!!" << std::endl;

    float x1 = 0.0;
    float x2 = 0.0;
    float x3 = 0.0;
    float x4 = 0.0;

    while (1) {// send loop
        to_mbed.ch1 = x1;
        to_mbed.ch2 = x2;
        to_mbed.ch3 = x3;
        to_mbed.ch4 = x4;
        x1 += 0.1;
        x2 += 0.2;
        x3 += 0.3;
        x4 += 0.4;
        sent_size = udp_server.send("", 12345, &to_mbed, sizeof(toMbed));
//    std::cout << "Sent size: " << sent_size << std::endl;
        SLEEP(10); // wait for 10ms
    }

    th1.get();
}
//#endif
