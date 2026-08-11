#ifndef CANBUS_H
#define CANBUS_H

#include "mbed.h"

class CANBus {
public:
    CANBus(PinName rd, PinName td);
    bool start(int bitrate);
    bool send(const CANMessage &message);
    bool read(CANMessage &message);
    void set_silent(bool silent);

    uint32_t get_bitrate() const { return bitrate; }
    uint32_t get_tx_count() const { return tx_count; }
    uint32_t get_rx_count() const { return rx_count; }
    uint8_t get_last_tx_error() const { return last_tx_error; }
    uint8_t get_last_rx_error() const { return last_rx_error; }

private:
    CAN can;
    bool started;
    uint32_t bitrate;
    uint32_t tx_count;
    uint32_t rx_count;
    uint8_t last_tx_error;
    uint8_t last_rx_error;
};

#endif // CANBUS_H
