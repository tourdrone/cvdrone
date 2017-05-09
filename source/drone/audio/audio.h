#ifndef AUDIO_HEADER
#define AUDIO_HEADER

// To gain access to these functions include audio.h

#include <string>

using namespace std;

// Issues the command to play a MP3 file.
void play_audio(string mp3_file_name);

// Kills all processes with ffplay in the name.
// For the purposes of the tour I recommend this be called prior to calling play_audio
// so that two or more audio files are not playing at the same time.
void kill_audio();

#endif
