/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "SerialConsole.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "Gcode.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "SDFAT.h"
#include "md5.h"

#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "PlayerPublicAccess.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "StepTicker.h"
#include "Block.h"
#include "quicklz.h"

#include <math.h>

#include <cstddef>
#include <cmath>
#include <algorithm>

#include "mbed.h"

#define home_on_boot_checksum             CHECKSUM("home_on_boot")
#define on_boot_gcode_checksum            CHECKSUM("on_boot_gcode")
#define on_boot_gcode_enable_checksum     CHECKSUM("on_boot_gcode_enable")
#define after_suspend_gcode_checksum      CHECKSUM("after_suspend_gcode")
#define before_resume_gcode_checksum      CHECKSUM("before_resume_gcode")
#define leave_heaters_on_suspend_checksum CHECKSUM("leave_heaters_on_suspend")
#define laser_module_clustering_checksum 	  CHECKSUM("laser_module_clustering")

extern SDFAT mounter;

extern unsigned char gotobuff[16];	//gotobuff[0] == 1: new data,gotobuff[0] == 0:old data; gotobuff[1~13]:length+cmdtype+played_lines(4byte)+played_cnt(4bytes)
extern const unsigned short crc_table[256];
// used for XMODEM
#define WAIT_MD5  0x01
#define WAIT_FILE_VIEW  0x02
#define READ_FILE_DATA  0x03

#define MAXRETRANS 50
#define RETRYTIME  50
#define TIMEOUT_MS 10
#define RETRYTIMES 10

#define MAXGCODEQUNE 224
#define PLAY_FINISH_REPORT_WINDOW_US 1000000UL //zqq modify 2026.06.09 keep |P:100%| visible for host polls

unsigned long g_request_lines=0;
volatile bool g_play_data_resend_pending = false; //zqq add 2026.05.20.
static bool bfilend = false; //zqq modify 2026.08.04
char queue_storage[14336] __attribute__((section("AHBSRAM1")));

StaticQueue gcode_buffer_queue(queue_storage, MAXGCODEQUNE, 64);

Player::Player()
{
    this->playing_file = false;
    this->current_file_handler = nullptr;
    this->booted = false;
    this->elapsed_secs = 0;
    this->reply_stream = nullptr;
    this->inner_playing = false;
    this->slope = 0.0;
    this->play_done_report = false; //zqq modify 2026.06.05
    this->play_done_lines = 0; //zqq modify 2026.06.05
    this->play_done_percent = 0; //zqq modify 2026.06.05
    this->play_done_elapsed = 0; //zqq modify 2026.06.05
    this->play_finish_cleanup_until = 0; //zqq modify 2026.06.09
    this->abort_progress_report_count = 0; //zqq modify 2026.08.03
}

void Player::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);

    this->on_boot_gcode = THEKERNEL->config->value(on_boot_gcode_checksum)->by_default("/sd/on_boot.gcode")->as_string();
    this->on_boot_gcode_enable = THEKERNEL->config->value(on_boot_gcode_enable_checksum)->by_default(false)->as_bool();

    this->home_on_boot = THEKERNEL->config->value(home_on_boot_checksum)->by_default(true)->as_bool();

    this->after_suspend_gcode = THEKERNEL->config->value(after_suspend_gcode_checksum)->by_default("")->as_string();
    this->before_resume_gcode = THEKERNEL->config->value(before_resume_gcode_checksum)->by_default("")->as_string();
    std::replace( this->after_suspend_gcode.begin(), this->after_suspend_gcode.end(), '_', ' '); // replace _ with space
    std::replace( this->before_resume_gcode.begin(), this->before_resume_gcode.end(), '_', ' '); // replace _ with space
    this->leave_heaters_on = THEKERNEL->config->value(leave_heaters_on_suspend_checksum)->by_default(false)->as_bool();

    this->laser_clustering = THEKERNEL->config->value(laser_module_clustering_checksum)->by_default(false)->as_bool();
}

void Player::on_halt(void* argument)
{
    this->clear_buffered_queue();

    if(argument == nullptr && this->playing_file ) {
        abort_command("1", &(StreamOutput::NullStream));
	}

	if(argument == nullptr && (THEKERNEL->is_suspending() || THEKERNEL->is_waiting())) {
		// clean up from suspend
		THEKERNEL->set_waiting(false);
		THEKERNEL->set_suspending(false);
		THEROBOT->pop_state();
		THEKERNEL->streams->printf("Suspend cleared\n");
	}
}

void Player::on_second_tick(void *)
{
    if(THEKERNEL->is_suspending() || THEKERNEL->is_tool_waiting()) {
        return;
    }

    if(this->playing_file) this->elapsed_secs++;
}

// extract any options found on line, terminates args at the space before the first option (-v)
// eg this is a file.gcode -v
//    will return -v and set args to this is a file.gcode
string Player::extract_options(string& args)
{
    string opts;
    size_t pos= args.find(" -");
    if(pos != string::npos) {
        opts= args.substr(pos);
        args= args.substr(0, pos);
    }

    return opts;
}

void Player::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    string args = get_arguments(gcode->get_command());
    if (gcode->has_m) {
        if (gcode->m == 21) { // Dummy code; makes Octoprint happy -- supposed to initialize SD card
#ifdef SDCARD
            mounter.remount();
            gcode->stream->printf("SD card ok\r\n");
#endif
        } else if (gcode->m == 23) { // select file
            this->filename = "/sd/" + args; // filename is whatever is in args
            this->current_stream = nullptr;

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }
            this->current_file_handler = fopen( this->filename.c_str(), "r");

            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
                return;

            } else {
                // get size of file
                int result = fseek(this->current_file_handler, 0, SEEK_END);
                if (0 != result) {
                    this->file_size = 0;
                } else {
                    this->file_size = ftell(this->current_file_handler);
                    fseek(this->current_file_handler, 0, SEEK_SET);
                }
                gcode->stream->printf("File opened:%s Size:%ld\r\n", this->filename.c_str(), this->file_size);
                gcode->stream->printf("File selected\r\n");
            }


            this->played_cnt = 0;
            this->played_lines = 0;
            this->request_lines = 0;
            this->elapsed_secs = 0;
            this->playing_lines = 0;
            this->goto_line = 0;
            this->play_cmd_tpye = 0;
            this->RemoteCrc = 0;
            g_request_lines = 0;
            g_play_data_resend_pending = false; //zqq add 2026.05.20.
            gcode_buffer_queue.clear();

        } else if (gcode->m == 24) { // start print
            if (this->current_file_handler != NULL) {
                this->abort_progress_report_count = 0; //zqq modify 2026.08.03
                this->playing_file = true;
                // this would be a problem if the stream goes away before the file has finished,
                // so we attach it to the kernel stream, however network connections from pronterface
                // do not connect to the kernel streams so won't see this FIXME
                this->reply_stream = THEKERNEL->streams;
            }

        } else if (gcode->m == 25) { // pause print
            this->playing_file = false;

        } else if (gcode->m == 26) { // Reset print. Slightly different than M26 in Marlin and the rest
            if(this->current_file_handler != NULL) {
                string currentfn = this->filename.c_str();
                unsigned long old_size = this->file_size;

                // abort the print
                abort_command("", gcode->stream);

                if(!currentfn.empty()) {
                    // reload the last file opened
                    this->current_file_handler = fopen(currentfn.c_str() , "r");

                    if(this->current_file_handler == NULL) {
                        gcode->stream->printf("file.open failed: %s\r\n", currentfn.c_str());
                    } else {
                        this->filename = currentfn;
                        this->file_size = old_size;
                        this->current_stream = nullptr;
                    }
                }
            } else {
                gcode->stream->printf("No file loaded\r\n");
            }

        } else if (gcode->m == 27) { // report print progress, in format used by Marlin
            progress_command("-b", gcode->stream);

        } else if (gcode->m == 32) { // select file and start print
            // Get filename
            this->filename = "/sd/" + args; // filename is whatever is in args including spaces
            this->current_stream = nullptr;

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }

            this->current_file_handler = fopen( this->filename.c_str(), "r");
            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
            } else {
                this->abort_progress_report_count = 0; //zqq modify 2026.08.03
                this->playing_file = true;

                // get size of file
                int result = fseek(this->current_file_handler, 0, SEEK_END);
                if (0 != result) {
                        file_size = 0;
                } else {
                        file_size = ftell(this->current_file_handler);
                        fseek(this->current_file_handler, 0, SEEK_SET);
                }
            }

            this->played_cnt = 0;
            this->played_lines = 0;
            this->request_lines = 0;
            this->elapsed_secs = 0;
            this->playing_lines = 0;
            this->goto_line = 0;
            this->play_cmd_tpye = 0;
            this->RemoteCrc = 0;
            g_request_lines = 0;
            g_play_data_resend_pending = false; //zqq add 2026.05.20.
            gcode_buffer_queue.clear();

        } else if (gcode->m == 600) { // suspend print, Not entirely Marlin compliant, M600.1 will leave the heaters on
            this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);

        } else if (gcode->m == 601) { // resume print
            this->resume_command("", gcode->stream);
        }

    }else if(gcode->has_g) {
        if(gcode->g == 28) { // homing cancels suspend
            if (THEKERNEL->is_suspending()) {
                // clean up
            	THEKERNEL->set_suspending(false);
                THEROBOT->pop_state();
            }
        }
    }
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Player::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    string possible_command = new_message.message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter(possible_command);

	// new_message.stream->printf("Play Received %s\r\n", possible_command.c_str());

    // Act depending on command
    if (cmd == "play"){
        this->play_command( possible_command, new_message.stream );
    }else if (cmd == "progress"){
        this->progress_command( possible_command, new_message.stream );
    }else if (cmd == "abort") {
        this->abort_command( possible_command, new_message.stream );
    }else if (cmd == "suspend") {
        this->suspend_command( possible_command, new_message.stream );
    }else if (cmd == "resume") {
        this->resume_command( possible_command, new_message.stream );
    }else if (cmd == "goto") {
    	this->goto_command( possible_command, new_message.stream );
    }else if (cmd == "buffer") {
    	this->buffer_command( possible_command, new_message.stream );
    }else if (cmd == "upload") {
    	this->upload_command( possible_command, new_message.stream );
    }else if (cmd == "download") {
        memset(md5_str, 0, sizeof(md5_str));
    	if (possible_command.find("config.txt") != string::npos) {
        	this->test_command( possible_command, new_message.stream );
    	}
    	this->download_command( possible_command, new_message.stream );
    }
}

// Buffer gcode to queue
void Player::buffer_command( string parameters, StreamOutput *stream )
{
	this->buffered_queue.push(parameters);
	stream->printf("Command buffered: %s\r\n", parameters.c_str());
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput *stream )
{
	if (!THEROBOT->is_homed_all_axes()) {
		return;
	}

    // extract any options from the line and terminate the line there
    string options= extract_options(parameters);
    // Get filename which is the entire parameter line upto any options found or entire line
    this->filename = absolute_from_relative(shift_parameter(parameters));
    this->last_filename = this->filename;

    if (this->playing_file || THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Currently printing, abort print first\r\n");
        return;
    }

    this->abort_progress_report_count = 0; //zqq modify 2026.08.03
    
    this->filenameCrc = crc16_ccitt(
        (unsigned char*)(this->filename.data()), 
        this->filename.length());

	safe_delay_us(300000);
    uint8_t request_data[2];
    request_data[0] = (this->filenameCrc >> 8) & 0xFF;
    request_data[1] = this->filenameCrc & 0xFF;
    THEKERNEL->serial->send_cfg_frame(PTYPE_PLAY_START, request_data, 2);
    
    // Output to the current stream if we were passed the -v ( verbose ) option
    if( options.find_first_of("Vv") == string::npos ) {
        this->current_stream = nullptr;
    } else {
        // we send to the kernels stream as it cannot go away
        this->current_stream = THEKERNEL->streams;
    }

}

// Goto a certain line when playing a file
void Player::goto_command( string parameters, StreamOutput *stream )
{	
	static uint32_t starttime = 0;
	unsigned short int crc;
	
    if (!THEKERNEL->is_suspending()) {
        stream->printf("Can only jump when pausing!\r\n");
        return;
    }

//    if (this->current_file_handler == NULL) {
//    	stream->printf("Missing file handle!\r\n");
//    	return;
//    }

    string line_str = shift_parameter(parameters);
    if (!line_str.empty()) {
        char *ptr = NULL;
        this->goto_line = strtol(line_str.c_str(), &ptr, 10);
        this->goto_line = this->goto_line < 1 ? 1 : this->goto_line;
        stream->printf("Goto line %lu...\r\n", this->goto_line);
        
		// send goto CMD request
	    uint8_t request_data[6];
	    request_data[0] = (this->filenameCrc >> 8) & 0xFF;
	    request_data[1] = this->filenameCrc & 0xFF;
	    request_data[2] = (this->goto_line >> 24) & 0xFF;
	    request_data[3] = (this->goto_line >> 16) & 0xFF;
	    request_data[4] = (this->goto_line >> 8) & 0xFF;
	    request_data[5] = this->goto_line & 0xFF;
	    THEKERNEL->serial->send_cfg_frame(PTYPE_GOTO_START, request_data, 6);

        // goto file begin
        //fseek(this->current_file_handler, 0, SEEK_SET);
        played_lines = 0;
        request_lines = 0;
        played_cnt   = 0;
        RemoteCrc = 0;
        g_request_lines = 0;
        g_play_data_resend_pending = false; //zqq add 2026.05.20.
        
		gcode_buffer_queue.clear();		//lsf add 2026.03.16

        const unsigned long completed_lines = this->goto_line - 1; //zqq modify 2026.08.04
        bool goto_response_received = false; //zqq modify 2026.08.04
        starttime = us_ticker_read();
        while (!goto_response_received) { //zqq modify 2026.08.04
        	if(gotobuff[0] == 1)
        	{
        		if(gotobuff[3] == PTYPE_GOTO_LINES)
				{
					crc = (gotobuff[4] <<8 ) | gotobuff[5];
					if(crc == this->filenameCrc)
					{
        				played_lines = (gotobuff[6]<<24) + (gotobuff[7]<<16) + (gotobuff[8]<<8) + gotobuff[9];
        				played_cnt = (gotobuff[10]<<24) + (gotobuff[11]<<16) + (gotobuff[12]<<8) + gotobuff[13];
						goto_response_received = (played_lines >= completed_lines); //zqq modify 2026.08.04
						starttime = us_ticker_read();
        			}
        		}
        		gotobuff[0] = 0;
        	}
        	
            THEKERNEL->call_event(ON_IDLE);
            if(us_ticker_read() - starttime > 1000000)	//time out
            {
        		g_request_lines = played_lines;		//lsf add 2026.03.16
            	stream->printf("Error:Goto line failed,current line is :%lu.\r\n", played_lines);         
            	return;
            }   	
        }
        g_request_lines = played_lines;		//lsf add 2026.03.16
        this->playing_lines = played_lines; //zqq modify 2026.08.04
        bfilend = false; //zqq modify 2026.08.04
        stream->printf("Info:Goto line successed,current line is :%lu.\r\n", this->goto_line); //zqq modify 2026.08.04
    }
}

void Player::progress_command( string parameters, StreamOutput *stream )
{

    // get options
    string options = shift_parameter( parameters );
    bool sdprinting= options.find_first_of("Bb") != string::npos;

    if(!playing_file && current_file_handler != NULL) {
        if(sdprinting)
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        else
            stream->printf("SD print is paused at %lu/%lu\r\n", played_cnt, file_size);
        return;

    } else if(!playing_file) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    if(file_size > 0) {
        unsigned long est = 0;
        if(this->elapsed_secs > 10) {
            unsigned long bytespersec = played_cnt / this->elapsed_secs;
            if(bytespersec > 0)
                est = (file_size - played_cnt) / bytespersec;
        }

        float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if (!sdprinting) {
            stream->printf("file: %s, %u %% complete, elapsed time: %02lu:%02lu:%02lu", this->filename.c_str(), (unsigned int)roundf(pcnt), this->elapsed_secs / 3600, (this->elapsed_secs % 3600) / 60, this->elapsed_secs % 60);
            if(est > 0) {
                stream->printf(", est time: %02lu:%02lu:%02lu",  est / 3600, (est % 3600) / 60, est % 60);
            }
            stream->printf("\r\n");
        } else {
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        }

    } else {
        stream->printf("File size is unknown\r\n");
    }
}

void Player::abort_command( string parameters, StreamOutput *stream )
{	
    if(!playing_file && current_file_handler == NULL) {
        stream->printf("Not currently playing\r\n");
        return;
    }

//    this->playing_file = false;
//    this->played_cnt = 0;
//    this->played_lines = 0;
//    this->playing_lines = 0;
    this->request_lines = 0;
    this->goto_line = 0;
//    this->file_size = 0;
    this->play_done_report = false; //zqq modify 2026.06.05
    this->play_finish_cleanup_until = 0; //zqq modify 2026.06.09
    this->clear_buffered_queue();
//    this->filename = "";
    this->current_stream = NULL;
    this->play_cmd_tpye = 0;
    this->RemoteCrc = 0;
    g_request_lines = 0;
    g_play_data_resend_pending = false; //zqq add 2026.05.20.
    gcode_buffer_queue.clear();
	
	THEKERNEL->serial->send_cfg_frame(PTYPE_PLAY_CAN, NULL, 0);
	
    fclose(current_file_handler);
    current_file_handler = NULL;

    THEKERNEL->set_suspending(false);
    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();

    // Save the final abort progress after all queued blocks have finished.
    this->playing_lines = this->played_lines;
    //zqq modify 2026.08.03
    this->play_done_lines = this->played_lines;
    this->play_done_percent = this->file_size > 0 ? roundf(((float)this->played_cnt * 100.0F) / this->file_size) : 0;
    this->play_done_elapsed = this->elapsed_secs;
    this->abort_progress_report_count = 3;
    // End the playing state only after the final abort snapshot is saved.
    this->playing_file = false;
    this->played_cnt = 0;
    this->played_lines = 0;
    this->file_size = 0;
    this->filename = "";

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    THEKERNEL->set_waiting(false);


    // turn off spindle
    {
		struct SerialMessage message;
		message.message = "M5";
		message.stream = THEKERNEL->streams;
		message.line = 0;
		THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    }

    if (parameters.empty()) {
        // clear out the block queue, will wait until queue is empty
        // MUST be called in on_main_loop to make sure there are no blocked main loops waiting to put something on the queue
        THEKERNEL->conveyor->flush_queue();

        // now the position will think it is at the last received pos, so we need to do FK to get the actuator position and reset the current position
        THEROBOT->reset_position_from_current_actuator_position();
        stream->printf("Aborted playing or paused file. \r\n");
    }
    
    if (THEKERNEL->get_laser_mode()) {		//lsf add 2026.03.20
    	struct SerialMessage message;
    	message.message = "laserabort";
		message.stream = THEKERNEL->streams;
		message.line = 0;
		THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    }

}

static bool is_empty_gcode_line(const string& data) //zqq add 2026.05.20.
{
    if (data.empty()) {
        return true; 
    }
    for (size_t i = 0; i < data.size(); i++) {
        char c = data[i];
        if (c != '\n' && c != '\r' && c != ' ' && c != '\t') {
            return false;
        }
    }
    return true; 
}

void Player::clear_buffered_queue(){
	while (!this->buffered_queue.empty()) {
		this->buffered_queue.pop();
	}
}

void Player::send_play_finish_snapshot_status() //zqq modify 2026.06.05
{
    StreamOutput *stream = THEKERNEL->serial;
    for(int i = 0; i < 3; i++) {
        std::string status = THEKERNEL->get_query_string();
        stream->PacketMessage(PTYPE_STATUS_RES, status.c_str(), 0); //zqq modify 2026.06.05
    }
}

void Player::cleanup_after_play_finish() //zqq modify 2026.06.09
{
    THEKERNEL->serial->send_cfg_frame(PTYPE_PLAY_END, NULL, 0); //zqq modify 2026.08.04
    this->play_finish_cleanup_until = 0;
    this->play_done_report = false;
    this->playing_file = false;
    this->filename = "";
    this->played_cnt = 0;
    this->played_lines = 0;
    this->request_lines = 0;
    this->goto_line = 0;
    this->file_size = 0;
    g_request_lines = 0;
    g_play_data_resend_pending = false;
    this->current_file_handler = NULL;
    this->current_stream = NULL;
}

void Player::on_main_loop(void *argument)
{
	unsigned short int crc;
	bool bnewline = false;
	static bool bplay_finish_pending = false; //zqq modify 2026.06.05
	unsigned short int datalen;
	static uint32_t starttime = 0;
	static unsigned long lastReqline = 100000;
	
    if( !this->booted ) {
        this->booted = true;
        if (this->home_on_boot) {
    		struct SerialMessage message;
    		message.message = "$H";
    		message.stream = THEKERNEL->streams;
    		message.line = 0;
    		THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
        }

        if (this->on_boot_gcode_enable) {
            this->play_command(this->on_boot_gcode, THEKERNEL->serial);
        }

    }

    if ( !this->playing_file ) {
    	
		if(this->play_cmd_tpye == PTYPE_PLAY_VIEW)
		{
			if(this->RemoteCrc == this->filenameCrc)
			{				
				THEKERNEL->streams->printf("  File size %ld\r\n", file_size);
				this->played_cnt = 0;
			    this->played_lines = 0;
			    this->request_lines = 0;
			    this->elapsed_secs = 0;
			    this->playing_lines = 0;
			    this->goto_line = 0;
            	this->RemoteCrc = 0;
			    bfilend = false;
			    bplay_finish_pending = false; //zqq modify 2026.06.05
			    this->play_done_report = false; //zqq modify 2026.06.05
			    this->play_finish_cleanup_until = 0; //zqq modify 2026.06.09
			    
            	g_request_lines = 0;
            	g_play_data_resend_pending = false; //zqq add 2026.05.20.
			
			    // force into absolute mode
			    THEROBOT->absolute_mode = true;
			    THEROBOT->e_absolute_mode = true;
			
			    // reset current position;
			    THEROBOT->reset_position_from_current_actuator_position();
			    
			    gcode_buffer_queue.clear();
			    
			    // send file data requeset
			    uint8_t request_data[8];
			    uint16_t buffsize;								//lsf add 2026.03.07
			    buffsize = MAXGCODEQUNE - gcode_buffer_queue.size() - 1;		//lsf add 2026.03.07
			    request_data[0] = (this->filenameCrc >> 8) & 0xFF;
			    request_data[1] = this->filenameCrc & 0xFF;
			    request_data[2] = (played_lines >> 24) & 0xFF;
			    request_data[3] = (played_lines >> 16) & 0xFF;
			    request_data[4] = (played_lines >> 8) & 0xFF;
			    request_data[5] = played_lines & 0xFF;
			    request_data[6] = (buffsize >> 8) & 0xFF;		//lsf add 2026.03.07
			    request_data[7] = buffsize & 0xFF;				//lsf add 2026.03.07
			    THEKERNEL->serial->send_cfg_frame(PTYPE_PLAY_DATA, request_data, 8);
			    
                this->abort_progress_report_count = 0; //zqq modify 2026.08.03
				this->playing_file = true;	
			}
			else
			{
				THEKERNEL->streams->printf("error: File name CRC check failed,please retry.\r\n");
			    THEKERNEL->serial->send_cfg_frame(PTYPE_PLAY_CAN, NULL, 0);
			}
			
		}
		this->play_cmd_tpye = 0;
    }
    else {
        if(this->play_finish_cleanup_until != 0) { //zqq modify 2026.06.09
            if(us_ticker_read() < this->play_finish_cleanup_until) {
                return;
            }
            this->cleanup_after_play_finish();
            return;
        }
        if(bplay_finish_pending) { //zqq modify 2026.06.05
            return;
        }
        if(THEKERNEL->is_halted() || THEKERNEL->is_suspending() || THEKERNEL->is_waiting() || this->inner_playing) {
            return;
        }

        // check if there are bufferd command
        while (!this->buffered_queue.empty()) {
        	THEKERNEL->streams->printf("%s\r\n", this->buffered_queue.front().c_str());
			struct SerialMessage message;
			message.message = this->buffered_queue.front();
			message.stream = THEKERNEL->streams;
			message.line = 0;
			this->buffered_queue.pop();

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
        	starttime = us_ticker_read();
            return;
        }
        
        if(!gcode_buffer_queue.empty()){
        
            struct SerialMessage message;
            char popdata[64];
            gcode_buffer_queue.pop(popdata, 64);
            popdata[63] = '\0';
            string data(popdata);
            if (!is_empty_gcode_line(data)) { //zqq add 2026.05.20. skip execute for empty line
	            message.message = data;
	            message.stream = this->current_stream == nullptr ? &(StreamOutput::NullStream) : this->current_stream;
	            message.line = played_lines + 1;
	            THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
	            played_cnt += data.size(); //zqq add 2026.05.20.
			}
            played_lines += 1; //zqq add 2026.05.20. empty line still counts played_lines
        }
        
        request_lines = g_request_lines;
		if((gcode_buffer_queue.size()<180) && !bfilend && \
        	( (request_lines != lastReqline )|| g_play_data_resend_pending || (us_ticker_read() - starttime > 5000000)) ) //zqq add 2026.05.20.
        {
    		uint8_t request_data[8];
		    uint16_t buffsize;								//lsf add 2026.03.07
		    
		    buffsize = MAXGCODEQUNE - gcode_buffer_queue.size() - 1;		//lsf add 2026.03.07
		    request_data[0] = (this->filenameCrc >> 8) & 0xFF;
		    request_data[1] = this->filenameCrc & 0xFF;
		    request_data[2] = (request_lines >> 24) & 0xFF;
		    request_data[3] = (request_lines >> 16) & 0xFF;
		    request_data[4] = (request_lines >> 8) & 0xFF;
		    request_data[5] = request_lines & 0xFF;
		    request_data[6] = (buffsize >> 8) & 0xFF;		//lsf add 2026.03.07
		    request_data[7] = buffsize & 0xFF;				//lsf add 2026.03.07
		    THEKERNEL->serial->send_cfg_frame(PTYPE_PLAY_DATA, request_data, 8);
		    

		    lastReqline = request_lines;
		    starttime = us_ticker_read();
		    g_play_data_resend_pending = false; //zqq add 2026.05.20.
        }
        
        			
		if(this->play_cmd_tpye == PTYPE_PLAY_END)
		{
			bfilend = true;
		}
		else if(this->play_cmd_tpye == PTYPE_PLAY_CAN)
		{
			this->playing_file = false;
			this->played_cnt = 0;
		    this->played_lines = 0;
		    this->request_lines = 0;
		    this->elapsed_secs = 0;
//		    this->playing_lines = 0;
		    this->goto_line = 0;
		    
            g_request_lines = 0;
            g_play_data_resend_pending = false; //zqq add 2026.05.20.
		    gcode_buffer_queue.clear();
		    bfilend = false;
		    bplay_finish_pending = false; 
		    this->play_done_report = false;
		    this->play_finish_cleanup_until = 0; //zqq modify 2026.06.09
		}
		this->play_cmd_tpye = 0;
        
		if(bfilend && gcode_buffer_queue.empty())
		{
			bplay_finish_pending = true; //zqq modify 2026.06.05
			THEKERNEL->conveyor->wait_for_idle(); 

            this->play_done_lines = g_request_lines; //zqq modify 2026.06.05
            this->play_done_percent = 100; 
            this->play_done_elapsed = this->elapsed_secs; 
            this->play_done_report = true; 
            this->send_play_finish_snapshot_status();
            this->play_finish_cleanup_until = us_ticker_read() + PLAY_FINISH_REPORT_WINDOW_US; //zqq modify 2026.06.09

	        if(this->reply_stream != NULL) {
                // if we were printing from an M command from pronterface we need to send this back
	            this->reply_stream->printf("Done printing file\r\n"); 
	            this->reply_stream = NULL; 
	        }

	        bool bbb = true; 
	        PublicData::set_value( atc_handler_checksum, set_job_complete_checksum, &bbb ); 
	        bfilend = false;
	        bplay_finish_pending = false; 
		}
    }
}

/*
bool Player::check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value)
{
	float new_slope = 0.0;
	bool is_cluster = false;
	Gcode *gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
	if (!gcode->has_m && gcode->has_g && gcode->g == 1) {
		*x_value = gcode->get_value('X');
		*y_value = gcode->get_value('Y');
		*s_value = gcode->get_value('S');
		*distance = sqrtf((*x_value) * (*x_value) + (*y_value) * (*y_value));
		if (*x_value == 0) {
			new_slope = *y_value > 0 ? 1000 : -1000;
		} else if (*y_value == 0) {
			new_slope = *x_value > 0 ? 0.001 : -0.001;
		} else {
			new_slope = *y_value / *x_value;
		}
		if ((*distance) < 1.0 && fabs (new_slope - *slope) < 0.1) {
			is_cluster = true;
		}
		*slope = new_slope;
	}
	delete gcode;

	return is_cluster;
}
*/

void Player::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(is_playing_checksum) || pdr->second_element_is(is_suspended_checksum)) {
        static bool bool_data;
        bool_data = pdr->second_element_is(is_playing_checksum) ? this->playing_file : THEKERNEL->is_suspending();
        pdr->set_data_ptr(&bool_data);
        pdr->set_taken();

    } else if(pdr->second_element_is(get_progress_checksum)) {
        static struct pad_progress p;
        if(this->abort_progress_report_count > 0) { //zqq modify 2026.08.03
            p.played_lines = this->play_done_lines;
            p.percent_complete = this->play_done_percent;
            p.elapsed_secs = this->play_done_elapsed;
            p.filename = this->last_filename;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
            this->abort_progress_report_count--;
        } else if(file_size > 0 && playing_file) {
            if(this->play_done_report) { //zqq modify 2026.06.05
                p.played_lines = this->play_done_lines;
                p.percent_complete = this->play_done_percent;
                p.elapsed_secs = this->play_done_elapsed;
            } else if (!this->inner_playing) {
                const Block *block = StepTicker::getInstance()->get_current_block();
                // Note to avoid a race condition where the block is being cleared we check the is_ready flag which gets cleared first,
                // as this is an interrupt if that flag is not clear then it cannot be cleared while this is running and the block will still be valid (albeit it may have finished)
                if (block != nullptr && block->is_ready && block->is_g123) {
                	this->playing_lines = block->line;
                	p.played_lines = this->playing_lines;
                } else {
//                	p.played_lines = this->played_lines;
                	p.played_lines = this->playing_lines;
                }
        	} else {
//        		p.played_lines = this->played_lines;
				p.played_lines = this->playing_lines;
        	}
            if(!this->play_done_report) { //zqq modify 2026.06.05
                p.elapsed_secs = this->elapsed_secs;
                float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
                p.percent_complete = roundf(pcnt);
            }
            p.filename = this->filename;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
        }
    } else if (pdr->second_element_is(inner_playing_checksum)) {
    	bool b = this->inner_playing;
        pdr->set_data_ptr(&b);
        pdr->set_taken();
    } else if (pdr->second_element_is(play_getline_checksum)) {
    	unsigned long *linenum = static_cast<uint32_t *>(pdr->get_data_ptr());
    	*linenum = this->request_lines;
        pdr->set_taken();
    }
}

void Player::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(abort_play_checksum)) {
        abort_command("", &(StreamOutput::NullStream));
        pdr->set_taken();
    } else if (pdr->second_element_is(inner_playing_checksum)) {
    	bool b = *static_cast<bool *>(pdr->get_data_ptr());
    	this->inner_playing = b;
    	if (this->playing_file) pdr->set_taken();
    } else if (pdr->second_element_is(restart_job_checksum)) {
    	if (!this->last_filename.empty()) {
    		THEKERNEL->streams->printf("Job restarted: %s.\r\n", this->last_filename.c_str());
        	this->play_command(this->last_filename, &(StreamOutput::NullStream));
    	}
    } else if (pdr->second_element_is(play_cmd_checksum)) {
    	unsigned char *t = static_cast<unsigned char*>(pdr->get_data_ptr());
    	this->play_cmd_tpye = t[0];
    } else if (pdr->second_element_is(play_RemoteCrc_checksum)) {
    	unsigned short int *t = static_cast<unsigned short int*>(pdr->get_data_ptr());
    	this->RemoteCrc = t[0];
    } else if (pdr->second_element_is(play_Filesize_checksum)) {
    	long *t = static_cast<long*>(pdr->get_data_ptr());
    	this->file_size = t[0];
    } else if (pdr->second_element_is(play_requestline_checksum)) {
    	this->request_lines ++;
    }
}

/**
Suspend a print in progress
1. send pause to upstream host, or pause if printing from sd
1a. loop on_main_loop several times to clear any buffered commmands
2. wait for empty queue
3. save the current position, extruder position, temperatures - any state that would need to be restored
4. retract by specifed amount either on command line or in config
5. turn off heaters.
6. optionally run after_suspend gcode (either in config or on command line)

User may jog or remove and insert filament at this point, extruding or retracting as needed

*/
void Player::suspend_command(string parameters, StreamOutput *stream )
{
    if (THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Already suspended!\n");
        return;
    }

    if(!this->playing_file) {
        stream->printf("Can not suspend when not playing file!\n");
        return;
    }

    stream->printf("Suspending , waiting for queue to empty...\n");
    
    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Suspend aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    this->playing_lines = this->played_lines;

    THEKERNEL->set_waiting(false);
    THEKERNEL->set_suspending(true);
    stream->printf("now save current pos...\n");
    // save current XYZ position in WCS
    Robot::wcs_t mpos= THEROBOT->get_axis_position();
    Robot::wcs_t wpos= THEROBOT->mcs2wcs(mpos);
    saved_position[0]= std::get<X_AXIS>(wpos);
    saved_position[1]= std::get<Y_AXIS>(wpos);
    saved_position[2]= std::get<Z_AXIS>(wpos);

    // save current state
    THEROBOT->push_state();
    current_motion_mode = THEROBOT->get_current_motion_mode();

    // execute optional gcode if defined
    if(!after_suspend_gcode.empty()) {
        struct SerialMessage message;
        message.message = after_suspend_gcode;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    THEKERNEL->streams->printf("Suspended, resume to continue playing\n");
}

/**
resume the suspended print
1. restore the temperatures and wait for them to get up to temp
2. optionally run before_resume gcode if specified
3. restore the position it was at and E and any other saved state
4. resume sd print or send resume upstream
*/
void Player::resume_command(string parameters, StreamOutput *stream )
{
    if(!THEKERNEL->is_suspending()) {
        stream->printf("Not suspended\n");
        return;
    }

    stream->printf("Resuming playing...\n");

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEROBOT->pop_state();
        THEKERNEL->set_suspending(false);
        return;
    }

    // execute optional gcode if defined
    if(!before_resume_gcode.empty()) {
        stream->printf("Executing before resume gcode...\n");
        struct SerialMessage message;
        message.message = before_resume_gcode;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    if (this->goto_line == 0) {
        // Restore position
        stream->printf("Restoring saved XYZ positions and state...\n");

        THEROBOT->absolute_mode = true;

        char buf[128];
        snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f Z%.3f F%.3f", saved_position[0], saved_position[1], saved_position[2], THEROBOT->from_millimeters(1000));
        struct SerialMessage message;
        message.message = buf;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );

    	if (current_motion_mode > 1) {
            snprintf(buf, sizeof(buf), "G%d", current_motion_mode - 1);
            message.message = buf;
            message.line = 0;
            THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    	}
    }

    THEROBOT->pop_state();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEKERNEL->set_suspending(false);
        return;
    }

	THEKERNEL->set_suspending(false);

	stream->printf("Playing file resumed\n");
}

unsigned int Player::crc16_ccitt(unsigned char *data, unsigned int len)
{
	unsigned char tmp;
	unsigned short crc = 0;

	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]) & 0xffff;
	}

	return crc & 0xffff;
}

int Player::check_crc(int crc, unsigned char *data, unsigned int len)
{
    if (crc) {
        unsigned short crc = crc16_ccitt(data, len);
        unsigned short tcrc = (data[len] << 8) + data[len+1];
        if (crc == tcrc)
            return 1;
    }
    else {
        unsigned char cks = 0;
        for (unsigned int i = 0; i < len; ++i) {
            cks += data[i];
        }
        if (cks == data[len])
        return 1;
    }

    return 0;
}

int Player::inbyte(StreamOutput *stream, unsigned int timeout_ms)
{
	uint32_t tick_us = us_ticker_read();
    while (us_ticker_read() - tick_us < timeout_ms * 1000) {
        if (stream->ready())
            return stream->_getc();
        safe_delay_us(100);
    }
    return -1;
}

int Player::inbytes(StreamOutput *stream, char **buf, int size, unsigned int timeout_ms)
{
	uint32_t tick_us = us_ticker_read();
    while (us_ticker_read() - tick_us < timeout_ms * 1000) {
        if (stream->ready())
            return stream->gets(buf, size);
        safe_delay_us(100);
    }
    return -1;
}

void Player::set_serial_rx_irq(bool enable)
{
	// disable serial rx irq
    bool enable_irq = enable;
    PublicData::set_value( atc_handler_checksum, set_serial_rx_irq_checksum, &enable_irq );
}
int Player::decompress(string sfilename, string dfilename, uint32_t sfilesize, StreamOutput* stream)
{

}

void Player::upload_command( string parameters, StreamOutput *stream )
{
}


void Player::test_command( string parameters, StreamOutput* stream ) {
    string filename = absolute_from_relative(shift_parameter(parameters));
	FILE *fd = fopen(filename.c_str(), "rb");
	if (NULL != fd) {
        MD5 md5;
        uint8_t md5_buf[64];
        do {
            size_t n = fread(md5_buf, 1, sizeof(md5_buf), fd);
            if (n > 0) md5.update(md5_buf, n);
            THEKERNEL->call_event(ON_IDLE);
        } while (!feof(fd));
        strcpy(md5_str, md5.finalize().hexdigest().c_str());
        fclose(fd);
        fd = NULL;
	}
}

void Player::download_command( string parameters, StreamOutput *stream )
{
}

void Player::SendMessage(char cmd, char* s, int size , StreamOutput *stream)
{	
}
