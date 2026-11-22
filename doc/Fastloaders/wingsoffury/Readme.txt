Wings of fury  game requires wingsoffury/1541.bin file to be dropped on the same directory as the game's D64 file and the following comands issued from BASIC once:

OPEN 1,8,15:PRINT#1,"XR:1541.BIN":CLOSE 1
OPEN 1,8,15:PRINT#1,"XW":CLOSE 1

The 1541.bin file contains a single signature byte required by the game to 
validate the 1541.