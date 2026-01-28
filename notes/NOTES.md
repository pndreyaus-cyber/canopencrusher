# 28.10.2025
1) Я меняю файлы OD.h и OD.c -- по идее нельзя их менять и вот надо будет как-то это решить
* I modify OD.h and OD.c. Well, I am not supposed to modify them, so that's something to solve
* Do we really need ODObjs_t?
* OD.c -> OD.cpp -- then we can compile Axis.cpp. Otherwise, there is an error, that we are trying to compile both C and C++ files

# 26.01
## Results
* Moved initialisation to setup()
* Moved can management into CanOpen
* Renamed MyCanOpen to CanOpen
* Resolved an algorithm of how to produce features in the future (alogrithms stays the same)
* Pushed to github and gitlab changes. Created and merged pull requests
* Loopback test before CAN initialisation

## What to do tomorrow (draft)
* Delete unused functions in the repo
* Clean "Robot" folder
* Change/Check, that CanOpen is used to implement functions like "set speed", "set target position", ...
* Using Perplexity define an algorithm for zero initialisation of motors. So that the motors think, that current position is 0
* Move Axis array somewhere (probably in MoveController), so that I do not explicitely have an array. Maybe use std::array
* Set constants to the same format
* Check is I can switch Serial2 to Serial 

# 27.01
## Results
* Cleaned folders in "Robot"
* Moved Axis management to MoveController
* Created a PR for axis management. Started solving problems, which Copilot gave me
* SYNC message is now sent
* The program does send some commands. But they require checking

## What to do tomorrow (draft)
* Finish solving problems for PR
* Merge PR to main
* Create a branch for feature/zeroInitializeEncoders. Implement this feature. Check it. Call it every time, the program is reset
* Start reading data from CAN