/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "ConfigValue.h"
#include "FileConfigSource.h"
#include "ConfigCache.h"
#include "checksumm.h"
#include "utils.h"
#include <malloc.h>
#include "SerialConsole.h"

using namespace std;
#include <string>
#include <string.h>

#define include_checksum     CHECKSUM("include")

extern const unsigned short crc_table[256];


FileConfigSource::FileConfigSource(string config_file, const char *name)
{
    this->name_checksum = get_checksum(name);
    this->config_file = config_file;
    this->config_file_found = false;
}

bool FileConfigSource::readLine(string& line, int lineno, FILE *fp)
{
    char buf[132];
    char *l= fgets(buf, sizeof(buf)-1, fp);
    if(l != NULL) {
        if(buf[strlen(l)-1] != '\n') {
            // truncate long lines
            if(lineno != 0) {
                // report if it is not truncating a comment
                if(strchr(buf, '#') == NULL)
                    printf("Truncated long line %d in: %s\n", lineno, config_file.c_str());
            }
            // read until the next \n or eof
            int c;
            while((c=fgetc(fp)) != '\n' && c != EOF) /* discard */;
        }
        line.assign(buf);
        return true;
    }

    return false;
}

// Transfer all values found in the file to the passed cache
void FileConfigSource::transfer_values_to_cache( ConfigCache *cache )
{
//    if( !this->has_config_file() ) {
//        return;
//    }
    transfer_values_to_cache( cache, this->get_config_file().c_str());
}
unsigned int crc16_ccitt(unsigned char *data, unsigned int len)
{
	unsigned char tmp;
	unsigned short crc = 0;

	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]) & 0xffff;
	}

	return crc & 0xffff;
}

uint8_t FileConfigSource::process_cfg_command(dfu_frame_t *frame, dfu_state_t *dfu_state, uint32_t *current_frame, uint32_t *total_frames, ConfigCache *cache) {
	char buf[130];
	
    switch (frame->type) {
        case PTYPE_CONFIG_START:
            // send PTYPE_FIRM_VIEW request 
            uint8_t response_data[6];
            response_data[0] = 0;
            response_data[1] = 0;
            response_data[2] = 0;
            response_data[3] = 0;
            response_data[4] = (132>>8)&0xff;
            response_data[5] = 132&0xff;
            THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_VIEW, response_data, 6);
            break;            
        case PTYPE_CONFIG_VIEW:
        	if (*dfu_state == DFU_STATE_START && frame->length >= 6) {
                *total_frames = (frame->data[0] << 24) | (frame->data[1] << 16) | (frame->data[2] << 8) | frame->data[3];
                
                *dfu_state = DFU_STATE_FILEVIEW;  
                
                uint8_t request_data[4];
                request_data[0] = (*current_frame + 1) >> 24;
                request_data[1] = (*current_frame + 1) >> 16;
                request_data[2] = (*current_frame + 1) >> 8;
                request_data[3] = (*current_frame + 1) & 0xFF;
                THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_DATA, request_data, 4);
            }
            break;            
        case PTYPE_CONFIG_DATA:
            if (*dfu_state == DFU_STATE_FILEVIEW || *dfu_state == DFU_STATE_TRANSFERRING) {
                *dfu_state = DFU_STATE_TRANSFERRING;
                
                if (frame->length >= 5) {
                    uint32_t frame_num = (frame->data[0] << 24) | (frame->data[1] << 16) | (frame->data[2] << 8) | frame->data[3];
                    
                    if (frame_num == *current_frame + 1) {
                        uint16_t data_len = frame->length - 4 - 3;
                        uint8_t *file_data = &frame->data[4];
                        
                        int line_start = 0;
                        
                        for (int i = 0; i <= data_len; i++) {
                        	if (i == data_len || file_data[i] == '\n' || file_data[i] == '\0') {
                        		if((i - line_start) == 0) // empty line
			            		{
			            			line_start = i+1;
			            			continue;
			            		}
                        		memset(buf, 0, sizeof(buf));
                        		memcpy(buf, &file_data[line_start], i-line_start+1);
                        		buf[i-line_start+1] = '\0';
								string line(reinterpret_cast<char*>(buf), sizeof(buf));
								process_line_from_ascii_config(line, cache);
								*current_frame = frame_num++;
								line_start = i+1;
                        	}
                        }
                         // 检查是否传输完成
                        if (*current_frame >= *total_frames) {
                            // 传输完成，发送PTYPE_FILE_END
                            THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_END, NULL, 0);
                            *dfu_state = DFU_STATE_IDLE;
							return 1;
                        } else {
                            // 请求下一帧数据
                            uint8_t request_data[4];
                            request_data[0] = (*current_frame + 1) >> 24;
                            request_data[1] = (*current_frame + 1) >> 16;
                            request_data[2] = (*current_frame + 1) >> 8;
                            request_data[3] = (*current_frame + 1) & 0xFF;
                            THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_DATA, request_data, 4);
                        }
                    }
                }
            } else {
                uint8_t error_code = 0x06;
                THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_CAN, &error_code, 1);
                *dfu_state = DFU_STATE_IDLE;
            }
            break;
            
        case PTYPE_CONFIG_END:            
        case PTYPE_CONFIG_CAN:
            THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_END, NULL, 0);
            *dfu_state = DFU_STATE_IDLE;
            return 1;
            break;
            
        default:
//            THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_CAN, NULL, 0);
//            *dfu_state = DFU_STATE_IDLE;
            break;
    }
	return 0;
}


void FileConfigSource::transfer_values_to_cache( ConfigCache *cache, const char * file_name )
{	
	dfu_frame_t frame;
	dfu_state_t dfu_state = DFU_STATE_IDLE;
	uint32_t total_frames = 0;
	uint32_t current_frame = 0;
	
    THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_START, NULL, 0);
    
    uint32_t wait_count = 0;
    const uint32_t MAX_WAIT_COUNT = 20;
    uint8_t start_ack_received = 0;
    while (wait_count < MAX_WAIT_COUNT && !start_ack_received) {
    	
        int result = THEKERNEL->serial->receive_cfg_frame(&frame, 100);
        
        if (result == 0) {
            process_cfg_command(&frame, &dfu_state, &current_frame, &total_frames, cache);
            
            if (frame.type == PTYPE_CONFIG_START) {
                start_ack_received = 1;
                dfu_state = DFU_STATE_START;
                continue;
            }
			else if (frame.type == PTYPE_CONFIG_CAN){
				return;
			}
        }
        
        wait_count++;
        if(wait_count % 5 == 0)
            THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_START, NULL, 0);
    }
	
    if (!start_ack_received) {
        THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_CAN, NULL, 0);
        return;
    }
	
	wait_count = 0;
    while (1) {
        int result = THEKERNEL->serial->receive_cfg_frame(&frame, 100);            
        if (result == 0) {
            if ((frame.type&0xf0) == 0xd0) {
				wait_count = 0;
				if(process_cfg_command(&frame, &dfu_state, &current_frame, &total_frames, cache)){
					return;
				}
			}        
        } else {
            wait_count++;
            if (wait_count > MAX_WAIT_COUNT) {
        		THEKERNEL->serial->send_cfg_frame(PTYPE_CONFIG_CAN, NULL, 0);
                break;
            }            
        }        
    }
}

/*
void FileConfigSource::transfer_values_to_cache( ConfigCache *cache, const char * file_name )
{
    if( !file_exists(file_name) ) {
        return;
    }

    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(file_name, "r");

    int ln= 1;
    // For each line
    while(!feof(lp)) {
        string line;
        if(readLine(line, ln++, lp)) {
            // process the config line and store the value in cache
            ConfigValue* cv = process_line_from_ascii_config(line, cache);

            if(cv == nullptr) continue;

            // if this line is an include directive then attempt to read the included file
            if(cv->check_sums[0] == include_checksum) {
                string inc_file_name = cv->value.c_str();
                cache->pop(); // we do not need to keep this around or leave it on the list

                if(!file_exists(inc_file_name)) {
                    // if the file is not found at the location entered then look around for it a bit
                    if(inc_file_name[0] != '/') inc_file_name = "/" + inc_file_name;
                    string path(file_name);
                    path = path.substr(0,path.find_last_of('/'));

                    // first check the path of the current config file
                    if(file_exists(path + inc_file_name)) inc_file_name = path + inc_file_name;
                    // then check root locations
                    else if(file_exists("/sd" + inc_file_name)) inc_file_name = "/sd" + inc_file_name;
                    else if(file_exists("/local" + inc_file_name)) inc_file_name = "/local" + inc_file_name;
                }
                if(file_exists(inc_file_name)) {
                    printf("Including config file: %s\n", inc_file_name.c_str());

                    // save position in current config file
                    fpos_t pos;
                    fgetpos(lp, &pos);

                    // open and read the included file
                    freopen(inc_file_name.c_str(), "r", lp);
                    this->transfer_values_to_cache(cache, inc_file_name.c_str());

                    // reopen the current config file and restore position
                    freopen(file_name, "r", lp);
                    fsetpos(lp, &pos);
                }else{
                    printf("Unable to find included config file: %s\n", inc_file_name.c_str());
                }
            }

        }else {
            break;
        }
    }
    fclose(lp);
}
*/
// Return true if the check_sums match
bool FileConfigSource::is_named( uint16_t check_sum )
{
    return check_sum == this->name_checksum;
}

// OverWrite or append a config setting to the file
bool FileConfigSource::write( string setting, string value )
{
    if( !this->has_config_file() ) {
        return false;
    }

    uint16_t setting_checksums[3];
    get_checksums(setting_checksums, setting );

    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(this->get_config_file().c_str(), "r+");

    // search each line for a match
    while(!feof(lp)) {
        string line;
        fpos_t bol, eol;
        fgetpos( lp, &bol ); // get start of line
        if(readLine(line, 0, lp)) {
            fgetpos( lp, &eol ); // get end of line
            if(!process_line_from_ascii_config(line, setting_checksums).empty()) {
                // found it
                unsigned int free_space = eol - bol - 4; // length of line
                // check we have enough space for this insertion
                if( (setting.length() + value.length() + 3) > free_space ) {
                    //THEKERNEL->streams->printf("ERROR: Not enough room for value\r\n");
                    fclose(lp);
                    return false;
                }

                // Update line, leaves whatever was at end of line there just overwrites the key and value
                fseek(lp, bol, SEEK_SET);
                fputs(setting.c_str(), lp);
                fputs(" ", lp);
                fputs(value.c_str(), lp);
                fputs(" #", lp);
                fclose(lp);
                return true;
            }
        }else break;
    }

    // not found so append the new value
    fclose(lp);
    lp = fopen(this->get_config_file().c_str(), "a");
    fputs("\n", lp);
    fputs(setting.c_str(), lp);
    fputs("         ", lp);
    fputs(value.c_str(), lp);
    fputs("         # added\n", lp);
    fclose(lp);

    return true;
}

// Return the value for a specific checksum
string FileConfigSource::read( uint16_t check_sums[3] )
{
    string value = "";

    if( this->has_config_file() == false ) {
        return value;
    }

    // Open the config file ( find it if we haven't already found it )
    FILE *lp = fopen(this->get_config_file().c_str(), "r");
    // For each line
    while(!feof(lp)) {
        string line;
         if(readLine(line, 0, lp)) {
            value = process_line_from_ascii_config(line, check_sums);
            if(!value.empty()) break; // found it
        }else break;
    }
    fclose(lp);

    return value;
}

// Return wether or not we have a readable config file
bool FileConfigSource::has_config_file()
{
    if( this->config_file_found ) {
        return true;
    }
    this->try_config_file(this->config_file);
    if( this->config_file_found ) {
        return true;
    } else {
        return false;
    }

}

// Tool function for get_config_file
inline void FileConfigSource::try_config_file(string candidate)
{
    if(file_exists(candidate)) {
        this->config_file_found = true;
    }
}

// Get the filename for the config file
string FileConfigSource::get_config_file()
{
    if( this->config_file_found ) {
        return this->config_file;
    }
    if( this->has_config_file() ) {
        return this->config_file;
    } else {
        printf("ERROR: no config file found\r\n");
        return "";
    }
}






