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
        
        enjoy !...
