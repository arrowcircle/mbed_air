#include <mbed.h>
#include "BME280.h"
#include "CANMsg.h"

Thread thread;
CAN can1(PA_11, PA_12);

void bme_thread() {
    BME280 sensor(I2C_SDA, I2C_SCL);
    while (true) {
        sensor.getTemperature();
        sensor.getPressure();
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
    can1.reset();
    can1.frequency(1000000);
    can1.mode(CAN::Normal);
    // can.filter(RX_ID, 0xFFF, CANStandard, 0); // set filter #0 to accept only standard messages with ID == RX_ID
    can1.attach(onCanReceived);

    thread.start(bme_thread);

    while (true) {
        ThisThread::sleep_for(500);
    }
}
