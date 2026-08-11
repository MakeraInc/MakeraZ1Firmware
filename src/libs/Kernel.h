/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KERNEL_H
#define KERNEL_H

#define THEKERNEL Kernel::instance
#define THECONVEYOR THEKERNEL->conveyor
#define THEROBOT THEKERNEL->robot

#include "Module.h"
#include "I2C.h" // mbed.h lib
#include <array>
#include <vector>
#include <string>
#include <malloc.h>

// 9 WCS offsets
#define MAX_WCS 9UL
#define CARVERA		1
#define CARVERA_AIR	2
#define MAKERA_Z1	3
#define MAKERA_Z1Pro	4

using std::string;
//Module manager
class Config;
class Module;
class Conveyor;
class SlowTicker;
class SerialConsole;
class StreamOutputPool;
class GcodeDispatch;
class Robot;
class Planner;
class StepTicker;
class Adc;
class PublicData;
class SimpleShell;
class Configurator;

enum STATE {
	IDLE    = 0,
	RUN     = 1,
	HOLD    = 2,
	HOME    = 3,
	ALARM   = 4,
	SLEEP   = 5,
	SUSPEND = 6,
	WAIT    = 7,
	TOOL    = 8
};

enum HALT_REASON {
	// No need to reset when triggered
	MANUAL     				= 1,
	HOME_FAIL  				= 2,
	PROBE_FAIL 				= 3,
	CALIBRATE_FAIL			= 4,
	ATC_HOME_FAIL   		= 5,
	ATC_TOOL_INVALID		= 6,
	ATC_NO_TOOL				= 7,
	ATC_HAS_TOOL			= 8,
	SPINDLE_OVERHEATED 		= 9,
	SOFT_LIMIT				= 10,
	COVER_OPEN				= 11,
	PROBE_INVALID			= 12,
	E_STOP					= 13,
	POWER_OVERHEATED		= 14,
	NON_HOME				= 15,
	// Need to reset when triggered
	HARD_LIMIT				= 21,
	MOTOR_ERROR_X			= 22,
	MOTOR_ERROR_Y			= 23,
	MOTOR_ERROR_Z			= 24,
	SPINDLE_STALL			= 25,
	SD_ERROR				= 26,
	// Need to switch off/on the power
	SPINDLE_ALARM			= 41
};

enum ATC_STATE {
	ATC_NONE   		= 0,
	ATC_DROP 		= 1,
	ATC_PICK		= 2,
	ATC_CALIBRATE	= 3,
	ATC_MARGIN		= 4,
	ATC_ZPROBE		= 5,
	ATC_AUTOLEVEL   = 6,
	ATC_DONE		= 9
};

typedef struct {
	float TLO;
	// int TOOL;
	float G54[3];
//	float G54[5*MAX_WCS];
	float REFMZ;
	float TOOLMZ;
	float reserve;
	int TOOL;
	float G54AB[2];
} EEPROM_data;

typedef struct {
	char  MachineModel;
	char  FuncSetting;
	char  reserve1;
	char  reserve2;
} FACTORY_SET;


enum ParseState { WAIT_HEADER, READ_LENGTH, READ_DATA, CHECK_FOOTER };

typedef struct {
    uint16_t header;      // ֡ͷ 0x8668
    uint16_t length;      // ���ݳ���
    uint8_t type;         // ֡����
    uint8_t data[544];    // ��������(���132�ֽ�)
    uint16_t crc;         // CRC16У��
    uint16_t tail;        // ֡β 0x55AA
} dfu_frame_t;

// DFU״̬����
typedef enum {
    DFU_STATE_IDLE = 0,        // ����״̬
    DFU_STATE_START,           // ��ʼ״̬
	DFU_STATE_FILEVIEW,		   // �ļ���Ҫ
    DFU_STATE_TRANSFERRING,    // ����״̬
    DFU_STATE_END,             // ����״̬
    DFU_STATE_ERROR            // ����״̬
} dfu_state_t;


#define PTYPE_CONFIG_START 0xd1
#define PTYPE_CONFIG_VIEW 0xd2
#define PTYPE_CONFIG_DATA 0xd3
#define PTYPE_CONFIG_END 0xd4
#define PTYPE_CONFIG_CAN 0xd5

#define PTYPE_FACTORY_START 0xe1
#define PTYPE_FACTORY_VIEW 0xe2
#define PTYPE_FACTORY_DATA 0xe3
#define PTYPE_FACTORY_END 0xe4
#define PTYPE_FACTORY_CAN 0xe5

#define PTYPE_PLAY_START 0xf1
#define PTYPE_PLAY_VIEW 0xf2
#define PTYPE_PLAY_DATA 0xf3
#define PTYPE_PLAY_END 0xf4
#define PTYPE_PLAY_CAN 0xf5
#define PTYPE_GOTO_START 0xf6
#define PTYPE_GOTO_LINES 0xf7


// support upload file type definition
#define FILETYPE	"lz"		//compressed by quicklz
// version definition
#define VERSION "1.1.2"

class Kernel {
    public:
        Kernel();

        ~Kernel() {
            delete this->i2c;
            delete this->eeprom_data;
            delete this->factory_set;
        }

        static Kernel* instance; // the Singleton instance of Kernel usable anywhere
        const char* config_override_filename(){ return "/sd/config-override"; }

        void add_module(Module* module);
        void register_for_event(_EVENT_ENUM id_event, Module *module);
        void call_event(_EVENT_ENUM id_event, void * argument= nullptr);

        bool kernel_has_event(_EVENT_ENUM id_event, Module *module);
        void unregister_for_event(_EVENT_ENUM id_event, Module *module);

        bool is_using_leds() const { return use_leds; }
        bool is_halted() const { return halted; }
        bool is_grbl_mode() const { return grbl_mode; }
        bool is_ok_per_line() const { return ok_per_line; }

        void set_feed_hold(bool f) { feed_hold= f; }
        bool get_feed_hold() const { return feed_hold; }
        bool is_feed_hold_enabled() const { return enable_feed_hold; }
        void set_bad_mcu(bool b) { bad_mcu= b; }
        bool is_bad_mcu() const { return bad_mcu; }

        void set_uploading(bool f) { uploading = f; }
        bool is_uploading() const { return uploading; }

        void set_laser_mode(bool f) { laser_mode = f; }
        bool get_laser_mode() const { return laser_mode; }

        void set_vacuum_mode(bool f) { vacuum_mode = f; }
        bool get_vacuum_mode() const { return vacuum_mode; }
        
        void set_blowing_mode(bool f) { blowing_mode = f; }
        bool get_blowing_mode() const { return blowing_mode; }
        
        void set_bedclean_mode(bool f) { bedclean_mode = f; }
        bool get_bedclean_mode() const { return bedclean_mode; }
        
        void set_elecremove_mode(bool f) { elecremove_mode = f; }
        bool get_elecremove_mode() const { return elecremove_mode; }

        void set_sleeping(bool f) { sleeping = f; }
        bool is_sleeping() const { return sleeping; }

        void set_suspending(bool f) { suspending = f; }
        bool is_suspending() const { return suspending; }

        void set_waiting(bool f) { waiting = f; }
        bool is_waiting() const { return waiting; }
        
        void set_tool_waiting(bool f) { tool_waiting = f; }
        bool is_tool_waiting() const { return tool_waiting; }

        void set_aborted(bool f) { aborted = f; }
        bool is_aborted() const { return aborted; }

        void set_zprobing(bool f) { zprobing = f; }
        bool is_zprobing() const { return zprobing; }
        
        void set_probeLaser(bool f) { probeLaserOn = f; }
        bool is_probeLaserOn() const { return probeLaserOn; }

        void set_halt_reason(uint8_t reason) { halt_reason = reason; }
        uint8_t get_halt_reason() const { return halt_reason; }

        void set_atc_state(uint8_t state) { atc_state = state; }
        uint8_t get_atc_state() const { return atc_state; }        
        
        void set_cachewait(bool f) { cachewait = f; }
        bool is_cachewait() const { return cachewait; }
        
        void set_PlaySpindleFan(bool f) { Play_SpindleFan = f; }
        bool is_PlaySpindleFan() const { return Play_SpindleFan; }
        
        void set_AutoClean(bool f) { b_AutoClean = f; }
        bool is_AutoClean() const { return b_AutoClean; }
        
        void set_3DProbeMode(bool f) {b_3DProbeMode = f;}
        bool is_3DProbeMode() const { return b_3DProbeMode; }
        
        void set_PlaySpindleFanValue(uint16_t f) { Play_SpindleFan_value = f; }
        uint16_t get_PlaySpindleFanValue() const { return Play_SpindleFan_value; }

        void read_eeprom_data();
        void write_eeprom_data();
        void erase_eeprom_data();
        void check_eeprom_data();
        
        void read_Factory_data();
        void write_Factory_data();
        void erase_Factory_data();
        void read_ProbeAddr_data();
        void write_ProbeAddr_Data_data(uint32_t Addr);
        void read_Factroy_SD();
        bool Check_Factory_Data(unsigned char *data, unsigned int len);
        bool Factroy_readLine(std::string& line, int lineno, FILE *fp);
        bool process_line(const std::string &buffer, uint16_t *check_sum, unsigned char *value);
        unsigned int crc16_ccitt(unsigned char *data, unsigned int len);

        std::string get_query_string();

        std::string get_diagnose_string();

        // These modules are available to all other modules
        SerialConsole*    serial;
        StreamOutputPool* streams;
        GcodeDispatch*    gcode_dispatch;
        Robot*            robot;
        Planner*          planner;
        Config*           config;
        Conveyor*         conveyor;
        Configurator*     configurator;
        SimpleShell*      simpleshell;

        SlowTicker*       slow_ticker;
        StepTicker*       step_ticker;
        Adc*              adc;
        std::string       current_path;
        uint32_t          base_stepping_frequency;
        uint32_t 		  power_24V_time;		//lsf add 2026.03.20

        uint8_t get_state();
        uint8_t halt_reason;
        uint8_t atc_state;
        EEPROM_data *eeprom_data;
        FACTORY_SET *factory_set;
        
        uint32_t Laser_period_us;        
        uint32_t Spindle_period_us;
        uint32_t probe_addr;
        uint16_t Play_SpindleFan_value;
        bool checkled;
        bool spindleon;
        bool axis_is_on[6];
    private:
        // When a module asks to be called for a specific event ( a hook ), this is where that request is remembered
        mbed::I2C* i2c;
        std::array<std::vector<Module*>, NUMBER_OF_DEFINED_EVENTS> hooks;
        struct {
            bool use_leds:1;
            bool halted:1;
            bool grbl_mode:1;
            bool feed_hold:1;
            bool ok_per_line:1;
            volatile bool enable_feed_hold:1;
            bool bad_mcu:1;
            volatile bool uploading:1;
            bool laser_mode:1;
            bool vacuum_mode:1;
            bool blowing_mode:1;
            bool bedclean_mode:1;
            bool elecremove_mode:1;
            bool sleeping:1;
            bool suspending: 1;
            bool waiting: 1;
            bool tool_waiting: 1;
            bool aborted: 1;
            bool zprobing:1;
            bool probeLaserOn:1;
            volatile bool cachewait:1;
            bool Play_SpindleFan:1;
            bool b_AutoClean:1;
            bool b_3DProbeMode:1;
        };
        int iic_page_write(unsigned char u8PageNum, unsigned char u8len, unsigned char *pu8Array);
        uint8_t process_factory_command(dfu_frame_t *frame, dfu_state_t *dfu_state, uint32_t *current_frame, uint32_t *total_frames, bool *bneedwrite);
        bool Factroy_ProcessLine(string& line, bool * bneedwrite);

};

#endif
