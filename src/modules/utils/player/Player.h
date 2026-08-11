/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "Module.h"
#include "mbed.h"

#include <stdio.h>
#include <string>
#include <map>
#include <vector>
#include <queue>
using std::string;
	
class StaticQueue {
private:
    struct QueueItem {
        char data[64];
        bool used;
    };
    
    QueueItem* items;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;
    
public:
    StaticQueue(void* memory, size_t item_count, size_t item_size) {
        items = static_cast<QueueItem*>(memory);
        capacity = item_count;
        head = tail = count = 0;
        
        for (size_t i = 0; i < capacity; i++) {
            items[i].used = false;
            items[i].data[0] = '\0';
        }
    }
    void clear() {
        head = tail = count = 0;
        for (size_t i = 0; i < capacity; i++) {
            items[i].used = false;
            items[i].data[0] = '\0';  // 可选：清空数据
        }
    }
    
    bool push(const char* str) {
        if (count >= capacity) return false;
        
        strncpy(items[tail].data, str, sizeof(items[tail].data) - 1);
        items[tail].data[sizeof(items[tail].data) - 1] = '\0';
        items[tail].used = true;
        
        tail = (tail + 1) % capacity;
        count++;
        return true;
    }
    
    bool pop(char* output, size_t max_len) {
        if (count == 0) return false;
        
        strncpy(output, items[head].data, max_len - 1);
        output[max_len - 1] = '\0';
        items[head].used = false;
        
        head = (head + 1) % capacity;
        count--;
        return true;
    }
    
    size_t size() const { return count; }
    bool empty() const { return count == 0; }
};

class StreamOutput;

class Player : public Module {
    public:
        Player();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_main_loop( void* argument );
        void on_second_tick(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_gcode_received(void *argument);
        void on_halt(void *argument);

    private:
        void play_command( string parameters, StreamOutput* stream );
        void progress_command( string parameters, StreamOutput* stream );
        void abort_command( string parameters, StreamOutput* stream );
        void suspend_command( string parameters, StreamOutput* stream );
        void resume_command( string parameters, StreamOutput* stream );
        void goto_command( string parameters, StreamOutput* stream );
        void buffer_command( string parameters, StreamOutput* stream );
        void upload_command( string parameters, StreamOutput* stream );
        void download_command( string parameters, StreamOutput* stream );
        void test_command(string parameters, StreamOutput* stream );
        string extract_options(string& args);

        void set_serial_rx_irq(bool enable);
        int inbyte(StreamOutput *stream, unsigned int timeout_ms);
        int inbytes(StreamOutput *stream, char **buf, int size, unsigned int timeout_ms);
        unsigned int crc16_ccitt(unsigned char *data, unsigned int len);
        int check_crc(int crc, unsigned char *data, unsigned int len);
		
		int decompress(string sfilename, string dfilename, uint32_t sfilesize, StreamOutput* stream);
//		int compressfile(string sfilename, string dfilename, StreamOutput* stream);
        // 2024
        // bool check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value);
        void SendMessage(char cmd, char* s, int size , StreamOutput *stream);

        string filename;
        string last_filename;
        string after_suspend_gcode;
        string before_resume_gcode;
        string on_boot_gcode;
        StreamOutput* current_stream;
        StreamOutput* reply_stream;

        char md5_str[64];

        std::queue<string> buffered_queue;
        void clear_buffered_queue();
        void send_play_finish_snapshot_status(); //zqq modify 2026.06.05
        void cleanup_after_play_finish(); //zqq modify 2026.06.09

        FILE* current_file_handler;
        // FILE* temp_file_handler;
        long file_size;
        unsigned long played_cnt;
        unsigned long elapsed_secs;
        unsigned long played_lines;
        unsigned long request_lines;
        unsigned long goto_line;
        unsigned int playing_lines;
        unsigned short int filenameCrc;
        unsigned short int RemoteCrc;
        uint8_t current_motion_mode;
        float saved_position[3]; // only saves XYZ
        float slope;
        std::map<uint16_t, float> saved_temperatures;
        unsigned char play_cmd_tpye;
        bool play_done_report; //zqq modify 2026.06.05
        unsigned long play_done_lines; //zqq modify 2026.06.05
        unsigned int play_done_percent; //zqq modify 2026.06.05
        unsigned long play_done_elapsed; //zqq modify 2026.06.05
        uint32_t play_finish_cleanup_until; //zqq modify 2026.06.09
        uint8_t abort_progress_report_count; //zqq modify 2026.08.03
        struct {
            bool on_boot_gcode_enable:1;
            bool booted:1;
            bool home_on_boot:1;
            bool playing_file:1;
            bool leave_heaters_on:1;
            bool override_leave_heaters_on:1;
            bool inner_playing:1;
            bool laser_clustering:1;
        };
};
