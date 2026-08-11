#include "CANBus.h"

CANBus::CANBus(PinName rd, PinName td)
    : can(rd, td), started(false), bitrate(0), tx_count(0), rx_count(0), last_tx_error(0), last_rx_error(0)
{
}

bool CANBus::start(int br)
{
    if (br <= 0) {
        return false;
    }

    bitrate = br;
    tx_count = 0;
    rx_count = 0;
    last_tx_error = 0;
    last_rx_error = 0;

    can.reset();
    if (can.frequency(bitrate) == 0) {
        return false;
    }
    started = true;
    return true;
}

bool CANBus::send(const CANMessage &message)
{
    if (!started) {
        return false;
    }

    if (can.write(message)) {
        ++tx_count;
        return true;
    }

    last_tx_error = can.tderror();
    return false;
}

bool CANBus::read(CANMessage &message)
{
    if (!started) {
        return false;
    }

    if (can.read(message)) {
        ++rx_count;
        return true;
    }

    last_rx_error = can.rderror();
    return false;
}

void CANBus::set_silent(bool silent)
{
    if (!started) {
        return;
    }

    can.monitor(silent);
}
