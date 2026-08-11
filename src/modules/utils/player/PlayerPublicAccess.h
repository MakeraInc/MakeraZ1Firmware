#ifndef PLAYERPUBLICACCESS_H
#define PLAYERPUBLICACCESS_H

#define player_checksum           CHECKSUM("player")
#define is_playing_checksum       CHECKSUM("is_playing")
#define is_suspended_checksum     CHECKSUM("is_suspended")
#define abort_play_checksum       CHECKSUM("abort_play")
#define get_progress_checksum     CHECKSUM("progress")
#define inner_playing_checksum    CHECKSUM("inner_playing")
#define restart_job_checksum      CHECKSUM("restart_job")

#define play_cmd_checksum		  CHECKSUM("play_cmd")
#define play_RemoteCrc_checksum	  CHECKSUM("play_remoteCrc")
#define play_Filesize_checksum	  CHECKSUM("play_filesize")
#define play_requestline_checksum CHECKSUM("play_requestline")
#define play_getline_checksum	  CHECKSUM("play_getcurlines")

struct pad_progress {
    unsigned int percent_complete;
    unsigned long played_lines;
    unsigned long elapsed_secs;
    std::string filename;
};
#endif
