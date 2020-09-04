#include <mbed.h>
#include "BME280.h"
#include "CANMsg.h"

#define USE_PLL_HSE_XTAL 1

Thread thread;
// CAN can1(RX, TX);
CAN can1(PA_11, PA_12);
// CAN can1(PA_11, PA_12);
bool can_ready = false;
DigitalOut led(PC_13);

void blink(int count, int total, int on) {
    for (int i = 0; i < count; i++) {
        led = 0;
        ThisThread::sleep_for(on);
        led = 1;
        ThisThread::sleep_for(total - on);
    }
}

void bme_thread() {
    BME280 sensor(PB_7, PB_6, 0x76);
    sensor.reset();
    CANMsg data;
    while (true) {
        if (can_ready != true) {
            continue;
        }

        sensor.trigger();

        while (sensor.busy() == true) {
            ThisThread::sleep_for(10);
        }

        float temperature = sensor.getTemperature();
        float pressure = sensor.getPressure();

        data.clear();
        data.id = 0x321;
        data << (int)pressure;

        bool res = true;
        if (!can1.write(data)) {
            res = false;
        }

        data.clear();
        data.id = 0x322;
        data << (int)temperature*100;

        if (can1.write(data) && res) {
            blink(1, 1000, 200);
        } else {
            blink(3, 300, 100);
        }
        
        ThisThread::sleep_for(1000);
    }
}

void onCanReceived(void) {
    CANMsg rxMsg;
    can1.read(rxMsg);
    // pc.printf("-------------------------------------\r\n");
    // pc.printf("CAN message received\r\n");
    // printMsg(rxMsg);
}

int main() {
    // can1.reset();
    can1.frequency(1000000);
    // can1.reset();
    can1.mode(CAN::Normal);
    // can.filter(RX_ID, 0xFFF, CANStandard, 0); // set filter #0 to accept only standard messages with ID == RX_ID
    can1.attach(onCanReceived);

    can_ready = true;
    thread.start(bme_thread);

    while (true) {
        ThisThread::sleep_for(5000);
    }
}
