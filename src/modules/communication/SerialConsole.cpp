/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <stdarg.h>
using std::string;
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "Ticker.h"
#include "SerialConsole.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "SerialConsole_DMA.h"
#include "..\utils\player\Player.h"
#include "..\utils\player\PlayerPublicAccess.h"

__attribute__((section("AHBSRAM0"), aligned(4))) char Serialbuff[544];
unsigned char gotobuff[16] __attribute__((section("AHBSRAM1")));	//gotobuff[0] == 1: new data,gotobuff[0] == 0:old data; gotobuff[1~13]:length+cmdtype+played_lines(4byte)+played_cnt(4bytes)
extern const unsigned short crc_table[256];
extern StaticQueue gcode_buffer_queue;

extern unsigned long g_request_lines;
extern volatile bool g_play_data_resend_pending; //zqq add 2026.05.20.


SerialConsole::SerialConsole( PinName rx_pin, PinName tx_pin, int baud_rate ){
    this->serial = new mbed::Serial( rx_pin, tx_pin );
    this->serial->baud(baud_rate);
    
    UART2_DMA_Init();    
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    // We want to be called every time a new char is received
    query_flag = false;
    halt_flag = false;
    diagnose_flag = false;
	this->attach_irq(true);

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams->append_stream(this);
}

void SerialConsole::attach_irq(bool enable_irq) {
//	if (enable_irq) {
//	    this->serial->attach(nullptr, mbed::Serial::RxIrq);
//	} else {
//	    this->serial->attach(nullptr, mbed::Serial::RxIrq);
//	}
}

void SerialConsole::on_set_public_data(void *argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_serial_rx_irq_checksum)) {
        bool enable_irq = *static_cast<bool *>(pdr->get_data_ptr());
        this->attach_irq(enable_irq);
        pdr->set_taken();
    }
}


// Called on Serial::RxIrq interrupt, meaning we have received a char
void SerialConsole::on_serial_char_received() {
	uint8_t headerBuffer[2];
    uint32_t received = 0;
    uint32_t timeout_ms = 100000;	//100 ms
    uint32_t starttime = 0;
    
    char buf[130];
	unsigned short int datalen;
	char *readbuf;
	int readlen = 0;
	unsigned long linenum = 0;
	unsigned short int RemoteCrc;
	long file_size;
	static uint32_t ComErrtime = 0;
    
    if (this->ready()) {
	    // wait head
	    starttime = us_ticker_read();
	    while ((received < 2) && ((us_ticker_read() - starttime) < timeout_ms) ) {
	        if (this->ready()) {
				headerBuffer[0] = headerBuffer[1];
				char byte = this->_getc();
				received++;
	            headerBuffer[1] = byte;
	            if (received >= 2 && (headerBuffer[0] != ((HEADER >> 8) & 0xFF) || 
	                                 headerBuffer[1] != (HEADER & 0xFF))) {
	                received = 1;
	            }
	        } 
	    }
	    
	    if (received < 2){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive header\r\n", 0);
	    	return;
	    }
	    
	    // receive length	    
	    starttime = us_ticker_read();
	    while ((received < 4) && ((us_ticker_read() - starttime) < timeout_ms) ) {
	        if (this->ready()) {
	        	char byte = this->_getc();
	        	Serialbuff[received] = byte;
	            received ++;
	        } 
	    }
	    
	    if (received < 4){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive length\r\n", 0);
	    	return;
	    }
	    
	    uint16_t data_len = (Serialbuff[2]<<8) | Serialbuff[3];
	    uint16_t total_len = 4 + data_len + 2; // header + data + crc + tail
	    
	    if (data_len > 513 || total_len > sizeof(Serialbuff)){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive datalen error\r\n", 0);
	    	 return; 
	    }
	    
	    starttime = us_ticker_read();
	    while ((received < total_len) && ((us_ticker_read() - starttime) < timeout_ms) ) {
	    	
	    	received += UART2_ReadData((uint8_t*)&Serialbuff[received], total_len-received);
	    	if(received < total_len)
	        	check_dma_progress_periodically();
	    }
	    
	    if (received < total_len) {
//	    		(PTYPE_NORMAL_INFO, "ALARM: Abort receive data body\r\n", 0);
	    	return;
	    }
	    
	    // check tail
	    uint16_t tail = (Serialbuff[total_len-2]<<8) | Serialbuff[total_len-1];
	    if (tail != FOOTER) {
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive footer\r\n", 0);
	    	return;
	    }
/*	    
	    // check CRC
	    uint16_t received_crc = (Serialbuff[total_len-4] << 8) | Serialbuff[total_len-3];
		uint16_t calculated_crc = crc16_ccitt((unsigned char *)&Serialbuff[2], data_len);
	    if (received_crc != calculated_crc) {
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive wrong crc\r\n", 0);
	        return;
	    }
*/	    
		ComErrtime = us_ticker_read();
		
	    uint8_t cmdType = Serialbuff[4];
        switch(cmdType) {
            case PTYPE_CTRL_SINGLE: { 
                if(Serialbuff[5] == '?') {
	            	query_flag = true;
	        	}
	        	else if(Serialbuff[5] == 'X' - 'A' + 1) {
	            	halt_flag = true;
	        	}
	        	else if(THEKERNEL->is_feed_hold_enabled()) {
		            if(Serialbuff[5] == '!') { // safe pause
		                THEKERNEL->set_feed_hold(true);
		            }
		            else if(Serialbuff[5] == '~') { // safe resume
		                THEKERNEL->set_feed_hold(false);
		            }
		        }
                break;
            }
            case PTYPE_CTRL_MULTI: {
                std::string received;
                received.assign(Serialbuff+5, data_len-3);
                if(received == "suspend" || received == "abort" || received == "resume" ||
                   received == "M600" || received == "M601" ||
                   received.compare(0, 4, "goto") == 0) {
                    deferred_command = received;
                } else {
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                }
                break;
            }
            case PTYPE_FILE_START: {
                struct SerialMessage message;
                message.message.assign(Serialbuff+5, data_len-3);
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                break;
            }
            case PTYPE_PLAY_START:
            	break;
            case PTYPE_PLAY_VIEW:
            	RemoteCrc = (Serialbuff[5] <<8 ) | Serialbuff[6];
            	file_size = (Serialbuff[7]<<24) | (Serialbuff[8]<<16) | (Serialbuff[9]<<8) | Serialbuff[10];
    			PublicData::set_value(player_checksum, play_RemoteCrc_checksum, &RemoteCrc);
    			PublicData::set_value(player_checksum, play_Filesize_checksum, &file_size);
    			PublicData::set_value(player_checksum, play_cmd_checksum, &cmdType);
    			break;
            case PTYPE_PLAY_END:
            case PTYPE_PLAY_CAN:
    			PublicData::set_value(player_checksum, play_cmd_checksum, &cmdType);
            	break;
            case PTYPE_PLAY_DATA:            	
            	linenum = (Serialbuff[7] << 24) | (Serialbuff[8] << 16) | (Serialbuff[9] << 8) | Serialbuff[10];
            	unsigned long request_lines;
//				PublicData::get_value( player_checksum, play_getline_checksum, &request_lines );
				if(linenum == g_request_lines)
				{
					g_play_data_resend_pending = false; //zqq add 2026.05.20.
					datalen = (Serialbuff[2]<<8) + Serialbuff[3];
					readbuf = (char*)(Serialbuff+11);
					readlen = datalen-9;
//					THEKERNEL->streams->printf("P_OK: ln=%lu qn=%lu rl=%d qsize=%u\r\n",
//						linenum, g_request_lines, readlen, (unsigned)gcode_buffer_queue.size());
					int len = readlen;            
		            if (len != 0){
		            
			            int line_start = 0;
			            for (int i = 0; i <= len; i++) {
			            	if (i == len || readbuf[i] == '\n') {
			            		if (i == len && line_start == len) {
			            			break; //zqq add 2026.05.20. skip phantom line after packet-ending '\n'
			            		}
			            		memset(buf, 0, sizeof(buf)); //zqq add 2026.05.20.
			            		if((i - line_start) == 0) {
			            			//zqq add 2026.05.20. empty line placeholder (counted as one line)
			            		} else {
				                memcpy(buf, &readbuf[line_start], i-line_start+1);
				                buf[i-line_start+1] = '\0';
			            		}
								if(!gcode_buffer_queue.push(buf)) //zqq add 2026.05.20. push empty/non-empty line
								{
									THEKERNEL->streams->printf("Alarm:push queue error at line %lu\n", g_request_lines);
								}
				                line_start = i+1;
				                g_request_lines += 1; //zqq add 2026.05.20. count empty line as one line
			            	}
			            }
			        }
				}
				else
				{
//					THEKERNEL->streams->printf("P_EL!!!: ln=%lu ql=%lu (mismatch)\r\n",
//						linenum, g_request_lines);
					g_play_data_resend_pending = true; //zqq add 2026.05.20.
				}
				
            	break;
            case PTYPE_GOTO_LINES:
				memset(gotobuff, 0, sizeof(gotobuff));
            	memcpy(gotobuff+1,Serialbuff+2,data_len+1);
            	gotobuff[0] = 1;
            	
            default:
            	break;
        }
	    
	}
	if((us_ticker_read() - ComErrtime) > 10000000)
	{
		ComErrtime = us_ticker_read();
		PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Ctrlboard did not receive a status response from the Mainboard (RX error)\r\n", 0);
	}
}


unsigned int SerialConsole::crc16_ccitt(unsigned char *data, unsigned int len)
{
	unsigned char tmp;
	unsigned short crc = 0;

	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]) & 0xffff;
	}

	return crc & 0xffff;
}

void SerialConsole::PacketMessage(char cmd, const char* s, int size)
{
	int crc = 0;
    unsigned int len = 0;
	size_t total_length = size == 0 ? strlen(s) : size;
	if(total_length>512) return;
	unsigned char *temp_buf = (unsigned char *)malloc(total_length + 9);
	temp_buf[0] = (HEADER>>8)&0xFF;
	temp_buf[1] = HEADER&0xFF;
	temp_buf[4] = cmd;
	
	memcpy(&temp_buf[5], s, total_length);
	len = total_length + 3;
	temp_buf[2] = (len>>8)&0xFF;
	temp_buf[3] = len&0xFF;
	crc = crc16_ccitt(&temp_buf[2], len);
	temp_buf[total_length+5] = (crc>>8)&0xFF;
	temp_buf[total_length+6] = crc&0xFF;
	temp_buf[total_length+7] = (FOOTER>>8)&0xFF;
	temp_buf[total_length+8] = FOOTER&0xFF;
	
	puts((char *)temp_buf, len+6);
	if (temp_buf != NULL){
        free(temp_buf);
    }
}

int SerialConsole::receive_cfg_frame(dfu_frame_t *frame, uint32_t timeout_ms) {	
	uint8_t headerBuffer[2];
    uint32_t received = 0;
    uint32_t starttime = 0;
    timeout_ms = 100000;	//100 ms
    // wait head
    starttime = us_ticker_read();
    while ((received < 2) && ((us_ticker_read() - starttime) < timeout_ms) ) {
        if (this->ready()) {
			headerBuffer[0] = headerBuffer[1];
			char byte = this->_getc();
			received++;
            headerBuffer[1] = byte;
            if (received >= 2 && (headerBuffer[0] != ((HEADER >> 8) & 0xFF) || 
                                 headerBuffer[1] != (HEADER & 0xFF))) {
                received = 1;
            }
        } 
    }
    
    if (received < 2){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive header\r\n", 0);
    	return -1;
    }
    
    
    // receive length
    starttime = us_ticker_read();
	    while ((received < 4) && ((us_ticker_read() - starttime) < timeout_ms) ) {
	        if (this->ready()) {
	        	char byte = this->_getc();
	        	Serialbuff[received] = byte;
	            received ++;
	        } 
	    }
	    
	    if (received < 4){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive length\r\n", 0);
	    	return -1;
	    }
    
    uint16_t data_len = (Serialbuff[2]<<8) | Serialbuff[3];
    uint16_t total_len = 4 + data_len + 2; // header + data + crc + tail
    
    if (data_len > 512 || total_len > sizeof(Serialbuff)) return -4; 
    
    starttime = us_ticker_read();
    while ((received < total_len) && ((us_ticker_read() - starttime) < timeout_ms) ) {
    	
    	received += UART2_ReadData((uint8_t*)&Serialbuff[received], total_len-received);
    	if(received < total_len)
        	check_dma_progress_periodically();
    }
    
    if (received < total_len) return -1;
    
    // check tail
    uint16_t tail = (Serialbuff[total_len-2]<<8) | Serialbuff[total_len-1];
    if (tail != FOOTER) return -2;
    
    // check CRC
    uint16_t received_crc = (Serialbuff[total_len-4] << 8) | Serialbuff[total_len-3];
	
    
    frame->header = HEADER;
    frame->length = data_len;
    frame->type = Serialbuff[4];
    if (data_len > 1 && data_len - 1 <= sizeof(frame->data)) {
        memcpy(frame->data, &Serialbuff[5], data_len - 1);
    } else if (data_len == 1) {
        frame->data[0] = 0; 
    }
    frame->crc = received_crc;
    frame->tail = FOOTER;
    uint16_t calculated_crc = crc16_ccitt((unsigned char *)&Serialbuff[2], data_len);
    if (received_crc != calculated_crc) {
        return -3;
    }
    
    return 0;
}
void SerialConsole::send_cfg_frame(uint8_t type, uint8_t *data, uint16_t len) {
	uint8_t sendbuf[128];
    uint16_t frame_len = 1 + len + 2; // type + data + crc
    uint16_t total_len = 2 + 2 + frame_len + 2; // header + length + data + tail
    
	if(total_len >128)
		return;
	uint8_t *frame = sendbuf;
    if (!frame) return;
    
    uint16_t offset = 0;
    
    frame[offset++] = (HEADER >> 8) & 0xFF;
    frame[offset++] = HEADER & 0xFF;
    
    frame[offset++] = (frame_len >> 8) & 0xFF;
    frame[offset++] = frame_len & 0xFF;
    
    frame[offset++] = type;
    
    if (data && len > 0) {
        memcpy(&frame[offset], data, len);
        offset += len;
    }
        
    uint16_t crc = crc16_ccitt(&frame[2], frame_len);
    frame[offset++] = (crc >> 8) & 0xFF;
    frame[offset++] = crc & 0xFF;
    
    frame[offset++] = (FOOTER >> 8) & 0xFF;
    frame[offset++] = FOOTER & 0xFF;
    
    puts((char *)frame, total_len);
    
}

int SerialConsole::printfcmd(const char cmd, const char *format, ...)
{
	char b[64];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);

    int size = vsnprintf(b, 64, format, args) + 1; // we add one to take into account space for the terminating \0

    if (size < 64) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }
    va_end(args);

//    puts(buffer, strlen(buffer));
	PacketMessage(PTYPE_DIAG_RES, buffer, strlen(buffer));

    if (buffer != b)
        delete[] buffer;

    return size - 1;
}

int SerialConsole::printf(const char *format, ...)
{
	char b[64];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);

    int size = vsnprintf(b, 64, format, args) + 1; // we add one to take into account space for the terminating \0

    if (size < 64) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }
    va_end(args);

	PacketMessage(PTYPE_NORMAL_INFO, buffer, strlen(buffer));

    if (buffer != b)
        delete[] buffer;

    return size - 1;
}


void SerialConsole::on_idle(void * argument)
{	
	static uint32_t starttime = 0;
    static uint32_t starttime2 = 0;
    static uint32_t startversion = 0;
    char buf[32];
    
//	if (THEKERNEL->is_uploading() || THEKERNEL->is_waiting()) return;
	
	on_serial_char_received();
	

    if (halt_flag) {
        halt_flag= false;
        THEKERNEL->set_halt_reason(MANUAL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        if(THEKERNEL->is_grbl_mode()) {
            PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort during cycle\r\n", 0);
        } else {
            PacketMessage(PTYPE_NORMAL_INFO, "HALTED, M999 or $X to exit HALT state\r\n", 0);
        }
    }
    
    if (query_flag) {
        query_flag= false;
        PacketMessage(PTYPE_STATUS_RES,THEKERNEL->get_query_string().c_str(),0);
    }
    
    
/* lsf modify 2026.03.17	
	if(us_ticker_read() - starttime > 300000){
        PacketMessage(PTYPE_STATUS_RES,THEKERNEL->get_query_string().c_str(),0);
        starttime = us_ticker_read();
    }

    if(us_ticker_read() - starttime2 > 500000){
    	PacketMessage(PTYPE_DIAG_RES,THEKERNEL->get_diagnose_string().c_str(),0);
        starttime2 = us_ticker_read();
    }
*/
    if(us_ticker_read() - startversion >5000000){
    	snprintf(buf, sizeof(buf), "%s", VERSION);
    	PacketMessage(PTYPE_FIRM_VER,buf,0);
    	startversion = us_ticker_read();
    }
}

// Actual event calling must happen in the main loop because if it happens in the interrupt we will loose data
void SerialConsole::on_main_loop(void * argument)
{    
    if(deferred_command.empty()) return;

    struct SerialMessage message;
    message.message = deferred_command;
    message.stream = this;
    message.line = 0;
    deferred_command.clear();
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
}

int SerialConsole::puts(const char* s, int size)
{
/*
    size_t n = size == 0 ? strlen(s) : size;
    return UART2_DMA_Send((uint8_t *)s, n);
*/    

    size_t n = size == 0 ? strlen(s) : size;
    for (size_t i = 0; i < n; ++i) {
        this->_putc(s[i]);
    }
    return n;

}

int SerialConsole::gets(char** buf, int size)
{

	return 0;
	
}

int SerialConsole::CheckFilePacket(char** buf) {
	
    return 0;
}

int SerialConsole::_putc(int c)
{
/*	Status status;
	status = UART2_DMA_Send((uint8_t*) &c, 1);
    if (status == SUCCESS)
    {
    	return 1;
    }
    else
    {
    	return -1;
    }
*/
   return this->serial->putc(c);
//	return UART2_DMA_Send((uint8_t*) &c, 1);
}

int SerialConsole::_getc()
{
	uint8_t data;
	UART2_ReadData(&data, 1);
    return (int)data;
}

bool SerialConsole::ready()
{	
	check_dma_progress_periodically();
    return GetRxAvailable();
}
