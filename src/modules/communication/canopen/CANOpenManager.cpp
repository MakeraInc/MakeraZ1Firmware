#include "CANOpenManager.h"

#include "Gcode.h"
#include "Kernel.h"
#include "Config.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"
#include "us_ticker_api.h"
#include "libs/utils.h"

#include <string>
#include <cstring>

#define canopen_checksum            CHECKSUM("canopen")
#define can_enable_checksum         CHECKSUM("can_enable")
#define can_bitrate_checksum        CHECKSUM("can_bitrate")
#define can_node_id_checksum        CHECKSUM("can_node_id")
#define slave_nid_checksum     		CHECKSUM("slave_node_id")
#define can_role_checksum           CHECKSUM("can_role")
#define can_heartbeat_checksum      CHECKSUM("can_heartbeat_ms")
#define can_status_checksum         CHECKSUM("status")
#define can_send_frame_checksum     CHECKSUM("send_frame")
static constexpr uint16_t CONFIG_INDEX = 0x6900;
static constexpr uint16_t DIGITAL_IN_INDEX = 0x6000;
static constexpr uint16_t DIGITAL_OUT_INDEX = 0x6001;

static uint32_t decode_baud_rate(uint32_t code)
{
    switch (code) {
        case 0: return 10000;
        case 1: return 20000;
        case 2: return 50000;
        case 3: return 125000;
        case 4: return 250000;
        case 5: return 500000;
        case 6: return 800000;
        case 7: return 1000000;
        default: return 0;
    }
}

static bool encode_baud_rate(uint32_t value, uint32_t &code)
{
    switch (value) {
        case 10000: code = 0; return true;
        case 20000: code = 1; return true;
        case 50000: code = 2; return true;
        case 125000: code = 3; return true;
        case 250000: code = 4; return true;
        case 500000: code = 5; return true;
        case 800000: code = 6; return true;
        case 1000000: code = 7; return true;
        default: return false;
    }
}

CANOpenManager::CANOpenManager()
    : enabled(false), master_mode(true), bus_ready(false), bitrate(1000000), node_id(1), heartbeat_ms(1000),
      last_heartbeat_timestamp(0), bus(nullptr), has_last_rx_message(false)
{
}

CANOpenManager::~CANOpenManager()
{
    shutdown_bus();
}

void CANOpenManager::on_module_loaded()
{
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_HALT);
    on_config_reload(this);
}

void CANOpenManager::on_config_reload(void *argument)
{
    enabled = THEKERNEL->config->value(canopen_checksum, can_enable_checksum)->by_default(true)->as_bool();
    bitrate = (uint32_t)THEKERNEL->config->value(canopen_checksum, can_bitrate_checksum)->by_default(1000000)->as_number();
    node_id = (uint8_t)THEKERNEL->config->value(canopen_checksum, can_node_id_checksum)->by_default(10)->as_number();
    slave_node_id = (uint8_t)THEKERNEL->config->value(canopen_checksum, slave_nid_checksum)->by_default(1)->as_number();
    heartbeat_ms = (uint32_t)THEKERNEL->config->value(canopen_checksum, can_heartbeat_checksum)->by_default(1000)->as_number();
    std::string role = THEKERNEL->config->value(canopen_checksum, can_role_checksum)->by_default("master")->as_string();
    master_mode = role != "slave";

    if (enabled) {
        initialize_bus();
    } else {
        shutdown_bus();
    }
}

bool CANOpenManager::initialize_bus()
{
    if (bus == nullptr) {
        bus = new CANBus(P0_4, P0_5);
    }

    bus_ready = bus->start(bitrate);
    if (!bus_ready) {
        THEKERNEL->streams->printf("CANopen: failed to start CAN controller at %lu bps\n", bitrate);
        return false;
    }

    last_heartbeat_timestamp = us_ticker_read();
    has_last_rx_message = false;
//    THEKERNEL->streams->printf("CANopen: controller ready (bitrate=%lu, node=%u, role=%s)\n", bitrate, node_id,
//                               master_mode ? "master" : "slave");
    return true;
}

void CANOpenManager::shutdown_bus()
{
    bus_ready = false;
    if (bus != nullptr) {
        delete bus;
        bus = nullptr;
    }
}

void CANOpenManager::on_main_loop(void *argument)
{
    if (!enabled || !bus_ready || bus == nullptr) {
        return;
    }

    poll_bus();

    uint32_t now = us_ticker_read();
    if (heartbeat_ms > 0) {
        send_heartbeat(now);
    }
}

void CANOpenManager::poll_bus()
{
    CANMessage message;
    while (bus->read(message)) {
        handle_passive_message(message);
    }
}

void CANOpenManager::send_heartbeat(uint32_t now_us)
{
    if (now_us - last_heartbeat_timestamp < heartbeat_ms * 1000) {
        return;
    }

    last_heartbeat_timestamp = now_us;
    uint8_t state = master_mode ? 0x05 : 0x7F; // Operational/Pre-operational default
    CANMessage heartbeat(0x700 + slave_node_id, reinterpret_cast<char *>(&state), 1, CANData, CANStandard);
    bus->send(heartbeat);
}

void CANOpenManager::print_status(StreamOutput *stream)
{
    stream->printf("CANopen %s\n", enabled ? "enabled" : "disabled");
    if (!enabled) {
        return;
    }

    stream->printf("  Bitrate: %lu\n", bitrate);
    stream->printf("  Node ID: %u\n", node_id);
    stream->printf("  Role: %s\n", master_mode ? "master" : "slave");
    stream->printf("  Heartbeat: %lu ms\n", heartbeat_ms);
    stream->printf("  Bus ready: %s\n", bus_ready ? "yes" : "no");
    if (bus_ready && bus != nullptr) {
        stream->printf("  TX: %lu RX: %lu\n", bus->get_tx_count(), bus->get_rx_count());
        if (has_last_rx_message) {
            stream->printf("  Last RX ID: 0x%03X LEN:%d\n", last_rx_message.id, last_rx_message.len);
        }
    }
}

void CANOpenManager::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if (!gcode->has_m) {
        return;
    }

    if (gcode->m == 940) {
        print_status(gcode->stream);
        gcode->stream->printf("  Last heartbeat at: %lu us\n", last_heartbeat_timestamp);
        gcode->stream->printf("ok\n");
    } else if (gcode->m == 941) {
        if (!enabled || !bus_ready || bus == nullptr) {
            gcode->stream->printf("error: CANopen disabled\n");
            return;
        }

        if (gcode->subcode == 0) {
	        if (!gcode->has_letter('X')) {
	            gcode->stream->printf("error: missing X (id)\n");
	            return;
	        }

	        uint32_t id = static_cast<uint32_t>(gcode->get_value('X'));
	        uint8_t len = 0;
	        uint8_t data[8] = {0};
	        if (gcode->has_letter('L')) {
	            len = static_cast<uint8_t>(gcode->get_value('L'));
	            if (len > 8) {
	                len = 8;
	            }
	        }
	        for (uint8_t i = 0; i < len; ++i) {
	            char letter = 'A' + i;
	            if (gcode->has_letter(letter)) {
	                data[i] = static_cast<uint8_t>(gcode->get_value(letter));
	            }
	        }

	        CANMessage msg(id, reinterpret_cast<char *>(data), len, CANData, CANStandard);
	        if (bus->send(msg)) {
	            gcode->stream->printf("ok\n");
	        } else {
	            gcode->stream->printf("error: send failed\n");
	        }
            return;
        }
        uint8_t target_node = resolve_node_id(gcode);
        uint32_t value = 0;
        uint8_t data_size = 0;
        switch (gcode->subcode) {
            case 1: {
                if (sdo_upload(target_node, CONFIG_INDEX, 1, value, data_size)) {
                    gcode->stream->printf("node %u id: %lu\n", target_node, value & 0xFFFFUL);
                    gcode->stream->printf("ok\n");
                } else {
                    gcode->stream->printf("error: failed to read node id\n");
                }
                return;
            }
            case 2: {
                if (!gcode->has_letter('X')) {
                    gcode->stream->printf("error: missing X (node id)\n");
                    return;
                }
                uint32_t new_id = static_cast<uint32_t>(gcode->get_value('X'));
                if (new_id < 1 || new_id > 127) {
                    gcode->stream->printf("error: invalid node id\n");
                    return;
                }
                if (sdo_download(target_node, CONFIG_INDEX, 1, new_id, 2)) {
                    gcode->stream->printf("node %u id set to %lu, power-cycle device to apply\n", target_node, static_cast<unsigned long>(new_id));
                    gcode->stream->printf("ok\n");
                } else {
                    gcode->stream->printf("error: failed to write node id\n");
                }
                return;
            }
            case 3: {
                if (sdo_upload(target_node, CONFIG_INDEX, 2, value, data_size)) {
                    uint32_t baud = decode_baud_rate(value & 0xFFFFUL);
                    if (baud > 0) {
                        gcode->stream->printf("node %u baud code: %lu (%lu bps)\n", target_node, value & 0xFFFFUL, static_cast<unsigned long>(baud));
                    } else {
                        gcode->stream->printf("node %u baud code: %lu\n", target_node, value & 0xFFFFUL);
                    }
                    gcode->stream->printf("ok\n");
                } else {
                    gcode->stream->printf("error: failed to read baud\n");
                }
                return;
            }
            case 4: {
                if (!gcode->has_letter('X')) {
                    gcode->stream->printf("error: missing X (baud)\n");
                    return;
                }
                uint32_t raw = static_cast<uint32_t>(gcode->get_value('X'));
                uint32_t baud_code = raw;
                if (raw > 15) {
                    if (!encode_baud_rate(raw, baud_code)) {
                        gcode->stream->printf("error: unsupported baud rate\n");
                        return;
                    }
                }
                if (sdo_download(target_node, CONFIG_INDEX, 2, baud_code, 2)) {
                    uint32_t baud = decode_baud_rate(baud_code);
                    if (baud > 0) {
                        gcode->stream->printf("node %u baud set to %lu bps (code %lu)\n", target_node, static_cast<unsigned long>(baud), static_cast<unsigned long>(baud_code));
                    } else {
                        gcode->stream->printf("node %u baud code set to %lu\n", target_node, static_cast<unsigned long>(baud_code));
                    }
                    gcode->stream->printf("ok\n");
                } else {
                    gcode->stream->printf("error: failed to set baud\n");
                }
                return;
            }
            case 5: {
                if (sdo_upload(target_node, DIGITAL_OUT_INDEX, 1, value, data_size)) {
                    gcode->stream->printf("node %u digital outputs: 0x%08lX\n", target_node, static_cast<unsigned long>(value));
                    gcode->stream->printf("ok\n");
                } else {
                    gcode->stream->printf("error: failed to read digital outputs\n");
                }
                return;
            }
            case 6: {
                if (!gcode->has_letter('X')) {
                    gcode->stream->printf("error: missing X (output value)\n");
                    return;
                }
                uint32_t outputs = static_cast<uint32_t>(gcode->get_value('X'));
                if (sdo_download(target_node, DIGITAL_OUT_INDEX, 1, outputs, 4)) {
                    gcode->stream->printf("node %u digital outputs set to 0x%08lX\n", target_node, static_cast<unsigned long>(outputs));
                    gcode->stream->printf("ok\n");
                } else {
                    gcode->stream->printf("error: failed to set digital outputs\n");
                }
                return;
            }
            case 7: {
                if (sdo_upload(target_node, DIGITAL_IN_INDEX, 1, value, data_size)) {
                    gcode->stream->printf("node %u digital inputs: 0x%08lX\n", target_node, static_cast<unsigned long>(value));
                    gcode->stream->printf("ok\n");
                } else {
                    gcode->stream->printf("error: failed to read digital inputs\n");
                }
                return;
            }
            default:
                gcode->stream->printf("error: unsupported M941.%d\n", gcode->subcode);
                return;
        }
    } else if (gcode->m == 942) {
    	uint32_t value = 0;
        uint8_t data_size = 0;
        
		gcode->stream->printf("start Can testing......\n");
		if (sdo_upload(0x01, DIGITAL_IN_INDEX, 1, value, data_size)) {
			sdo_download(0x01, DIGITAL_OUT_INDEX, 1, value, 4);
		}
		safe_delay_us(10000000);
		sdo_download(0x01, DIGITAL_OUT_INDEX, 1, 0, 4);
		gcode->stream->printf("Can test finish......\n");	
    }
}

void CANOpenManager::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);
    if (!pdr->starts_with(canopen_checksum)) {
        return;
    }

    if (pdr->second_element_is(can_status_checksum)) {
        static CANStatus status;
        status.enabled = enabled;
        status.bus_ready = bus_ready;
        status.bitrate = bitrate;
        status.node_id = node_id;
        status.heartbeat_ms = heartbeat_ms;
        status.tx_count = (bus_ready && bus != nullptr) ? bus->get_tx_count() : 0;
        status.rx_count = (bus_ready && bus != nullptr) ? bus->get_rx_count() : 0;
        status.has_last_message = has_last_rx_message;
        if (has_last_rx_message) {
            status.last_message = last_rx_message;
        }
        pdr->set_data_ptr(&status);
        pdr->set_taken();
    }
}

void CANOpenManager::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);
    if (!pdr->starts_with(canopen_checksum)) {
        return;
    }

    if (!pdr->second_element_is(can_send_frame_checksum)) {
        return;
    }

    CANFrame *frame = static_cast<CANFrame *>(pdr->get_data_ptr());
    if (frame == nullptr || !enabled || !bus_ready || bus == nullptr) {
        return;
    }

    uint8_t len = frame->length > 8 ? 8 : frame->length;
    CANMessage msg(frame->id, reinterpret_cast<char *>(frame->data), len,
                   frame->rtr ? CANRemote : CANData,
                   frame->extended ? CANExtended : CANStandard);
    if (bus->send(msg)) {
        pdr->set_taken();
    }
}

bool CANOpenManager::wait_for_sdo_response(uint16_t cob_id, CANMessage &response, uint32_t timeout_us)
{
    if (!bus_ready || bus == nullptr) {
        return false;
    }
    uint32_t start = us_ticker_read();
    while (true) {
        if (bus->read(response)) {
            if (response.id == cob_id) {
                return true;
            }
            handle_passive_message(response);
            continue;
        }
        if (us_ticker_read() - start >= timeout_us) {
            break;
        }
    }
    return false;
}
bool CANOpenManager::sdo_upload(uint8_t target_node, uint16_t index, uint8_t subindex, uint32_t &value, uint8_t &size, uint32_t timeout_us)
{
    if (!bus_ready || bus == nullptr) {
        return false;
    }

    char payload[8] = {0};
    payload[0] = 0x40;
    payload[1] = index & 0xFF;
    payload[2] = index >> 8;
    payload[3] = subindex;

    CANMessage request(0x600 + target_node, payload, 8, CANData, CANStandard);
    if (!bus->send(request)) {
        return false;
    }

    CANMessage response;
    if (!wait_for_sdo_response(0x580 + target_node, response, timeout_us)) {
        return false;
    }

    uint8_t command = response.data[0];
    if ((command & 0xE0) != 0x40) {
        return false;
    }

    bool expedited = (command & 0x02) != 0;
    if (!expedited) {
        return false;
    }

    bool size_indicated = (command & 0x01) != 0;
    uint8_t unused = (command >> 2) & 0x03;
    uint8_t data_size = size_indicated ? (4 - unused) : 4;
    if (data_size == 0) {
        data_size = 4;
    }

    value = 0;
    std::memcpy(&value, &response.data[4], data_size);
    size = data_size;
    return true;
}

bool CANOpenManager::sdo_download(uint8_t target_node, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size, uint32_t timeout_us)
{
    if (!bus_ready || bus == nullptr || size == 0 || size > 4) {
        return false;
    }

    char payload[8] = {0};
    uint8_t unused = 4 - size;
    payload[0] = 0x23 | (unused << 2);
    payload[1] = index & 0xFF;
    payload[2] = index >> 8;
    payload[3] = subindex;
    std::memcpy(&payload[4], &value, size);

    CANMessage request(0x600 + target_node, payload, 8, CANData, CANStandard);
    if (!bus->send(request)) {
        return false;
    }

    CANMessage response;
    if (!wait_for_sdo_response(0x580 + target_node, response, timeout_us)) {
        return false;
    }

    if (response.data[0] != 0x60) {
        return false;
    }

    return true;
}

uint8_t CANOpenManager::resolve_node_id(Gcode *gcode) const
{
    if (gcode != nullptr && gcode->has_letter('N')) {
        int requested = static_cast<int>(gcode->get_value('N'));
        if (requested >= 1 && requested <= 127) {
            return static_cast<uint8_t>(requested);
        }
    }

    return slave_node_id;
}

void CANOpenManager::handle_passive_message(const CANMessage &message)
{
    last_rx_message = message;
    has_last_rx_message = true;
}
void CANOpenManager::on_halt(void *argument)
{
    if (argument == nullptr) {
        return;
    }

    has_last_rx_message = false;
}
