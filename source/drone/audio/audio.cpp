/*
    Min Kao Drone Tour
    Created by Jacob Pollack
*/

#include <iostream>
#include <string>
#include <fstream>
#include "audio.h"

using namespace std;

// Issues the command to play a MP3 file.
void play_audio(string mp3_file_name){
  string audio_file = "../../tour_audio_files/" + mp3_file_name;
  ifstream infile(audio_file);
  if(infile.good()){
    system(("ffplay -nodisp -autoexit " + audio_file + " </dev/null >/dev/null 2>/dev/null &").c_str());
  }
  else{
    cerr << "ERROR: cannot open " << audio_file << endl;
  }
}

// Kills all processes with ffplay in the name.
// For the purposes of the tour I recommend this be called prior to calling play_audio
// so that two or more audio files are not playing at the same time.
void kill_audio(){
  system("pkill -f ffplay");
}
