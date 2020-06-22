#include <cstring>
#include <stdarg.h>
#include <thread>
#include <iostream>
#include <sstream>

#include "LpmsIG1I.h"
#include "SensorDataI.h"
#include "LpmsIG1Registers.h"

using namespace std;
string TAG("Main");
IG1I* sensor1;
// Start print data thread
std::thread *printThread;
static bool printThreadIsRunning = false;

void logd(std::string tag, const char* str, ...)
{
    va_list a_list;
    va_start(a_list, str);
    if (!tag.empty())
        printf("[%s] ", tag.c_str());
    vprintf(str, a_list);
    va_end(a_list);
}

void printTask()
{
    while (sensor1->getStatus() != STATUS_DISCONNECTED && printThreadIsRunning)
    {
        sensor1->sendCommand(GET_IMU_DATA, 0, NULL);

        IG1ImuDataI sd;
        sensor1->getImuData(sd);
        float freq = sensor1->getDataFrequency() ;
        //logd(TAG, "t:%d %.3f eulerX: %.2f eulerY: %.2f eulerZ: %.2f Hz:%.3f\r\n", sd.timestamp, sd.timestamp*0.002f, sd.euler.data[0], sd.euler.data[1], sd.euler.data[2], freq);
        logd(TAG, "t:%d %.3f qW: %.2f qX: %.2f qY: %.2f qZ: %.2f Hz:%.3f\r\n", sd.timestamp, sd.timestamp*0.002f, sd.quaternion.data[0], sd.quaternion.data[1], sd.quaternion.data[2], sd.quaternion.data[3], freq);

        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

void printMenu()
{
    cout << "Main Menu" << endl;
    cout << "===================" << endl;
    cout << "[h] Reset sensor heading" << endl;
    cout << "[i] Print sensor info" << endl;
    cout << "[s] Print sensor settins" << endl;
    cout << "[p] Print sensor data" << endl;
    cout << "[q] quit" << endl;
    cout << endl;
}

int main(int argc, char** argv)
{

    string comportNo = "/dev/ttyTHS2";

//     int baudrate = 115200;
    int baudrate = 230400;

    // Create LpmsIG1 object with corresponding comport and baudrate
    sensor1 = IG1Factory();
    sensor1->setConnectionInterface(COMMUNICATION_INTERFACE_485);
    sensor1->setControlGPIOForRs485(388);
    sensor1->setControlGPIOToggleWaitMs(400);

    cout << "connecting to sensor\r\n";
    // Connects to sensor
    if (!sensor1->connect(comportNo, baudrate))
    {
        logd(TAG, "Error connecting to sensor\n");
        sensor1->release();
        this_thread::sleep_for(chrono::milliseconds(1000));
        return 0;
    }

    do
    {
        logd(TAG, "Wait for sensor connected\r\n");
        this_thread::sleep_for(chrono::milliseconds(100));
    }while(!(sensor1->getStatus() == STATUS_CONNECTED));

    printMenu();

    bool quit = false;
    while (!quit)
    {
        char cmd;
        cin >> cmd;
        switch (cmd)
        {
        case 'h':
            // reset sensor heading
            sensor1->commandSetOffsetMode(LPMS_OFFSET_MODE_HEADING);
            break;

        case 'i': {
            // Print sensor info
            IG1InfoI info;
            sensor1->getInfo(info);
            cout << info.toString() << endl;
            break;
        }

        case 's': {
            // Print sensor settings
            IG1SettingsI settings;
            sensor1->getSettings(settings);
            cout << settings.toString() << endl;
            break;
        }

        case 'p': {
            // Print sensor data
            printThreadIsRunning = !printThreadIsRunning;
            printThread = new std::thread(printTask);
            break;
        }

        case 'q':
            // disconnect sensor
            sensor1->disconnect();
            quit = true;
            break;

        default:
            printThreadIsRunning = false;
            break;
        }
        this_thread::sleep_for(chrono::milliseconds(100));
    }


    printThread->join();
    // release sensor resources
    sensor1->release();
    logd(TAG, "Bye\n");
}
