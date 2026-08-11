#ifndef CANOPENMANAGER_H
#define CANOPENMANAGER_H

#include "Module.h"
#include "CANBus.h"
#include <cstdint>

class StreamOutput;
class Gcode;
class CANOpenManager : public Module {
public:
    struct CANStatus {
        bool enabled;
        bool bus_ready;
        uint32_t bitrate;
        uint8_t node_id;
        uint32_t heartbeat_ms;
        uint32_t tx_count;
        uint32_t rx_count;
        bool has_last_message;
        CANMessage last_message;
    };

    struct CANFrame {
        uint32_t id;
        uint8_t length;
        uint8_t data[8];
        bool rtr;
        bool extended;
    };

    CANOpenManager();
    ~CANOpenManager() override;

    void on_module_loaded() override;
    void on_main_loop(void *argument) override;
    void on_gcode_received(void *argument) override;
    void on_get_public_data(void *argument) override;
    void on_set_public_data(void *argument) override;
    void on_halt(void *argument) override;
    void on_config_reload(void *argument);

private:
    bool initialize_bus();
    void shutdown_bus();
    void poll_bus();
    void send_heartbeat(uint32_t now_us);
    void print_status(StreamOutput *stream);
    bool wait_for_sdo_response(uint16_t cob_id, CANMessage &response, uint32_t timeout_us = 50000);
    bool sdo_upload(uint8_t target_node, uint16_t index, uint8_t subindex, uint32_t &value, uint8_t &size, uint32_t timeout_us = 50000);
    bool sdo_download(uint8_t target_node, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size, uint32_t timeout_us = 50000);
    uint8_t resolve_node_id(Gcode *gcode) const;
    void handle_passive_message(const CANMessage &message);

    bool enabled;
    bool master_mode;
    bool bus_ready;
    uint32_t bitrate;
    uint8_t node_id;
    uint8_t slave_node_id;
    uint32_t heartbeat_ms;
    uint32_t last_heartbeat_timestamp;
    CANBus *bus;
    CANMessage last_rx_message;
    bool has_last_rx_message;
};

#endif // CANOPENMANAGER_H
