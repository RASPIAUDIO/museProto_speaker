# museProto_speaker
Here is a tiny speaker app using a Muse Proto board

with 2 running modes:
   - bluetooth player (BT ID : MUSE-SPEAKER-xxxxxx)
   - SD player(mp3)
   
It uses 3 buttons:
   - VM (gpio 32)
        - short press => volume -
        - long press => backward (SD mode)
   - VP (gpio 19)
        - short press => volume +
        - long press => forward (Sd mode)
   - MU (gpio 12)
        - short press => mute/unmute
        - very long press => stop (deep sleep) / restart
        
## Before building it using Arduino...

   1. You have to add  the Muse specific library (muse_lib) to Arduino libraries
   
   	For example using these bash commands :
   
             > cd ..../museProto_radio
             > cp -r muse_lib ..../Arduino/libraries
             
   2. You have also to copy some audio files from the "data" directory to the flash memory
   
             with Arduino : => Tools => ESP32 Sketch Data Upload
