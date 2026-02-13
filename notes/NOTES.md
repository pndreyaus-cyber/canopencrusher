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

# 28.01
## Results
* Finished PR with moving axes management to movecontroller
* Made a PR for zero initialization command
    * It is handled from Serial
    * Checked for correctness: length, parameter values, number of parameters
    * Zero-initialization commands sent to CAN bus

## What to do tomorrow
* Start reading data from CAN
* Read motor positions at setup and set axes positions to the consecutive positions
* Check/modify code, so that functions have the same return style (return boolean or int to show the result of running this function)

# 29.01
## Results
* Finished PR with zero intialization
* Implemented "RPP" command to request position
* Version 1 of reading from CAN bus
* Parsing of heartbeat and position messages
* Made a PR for reading feature

## What to do tomorrow
* 2-3 times request a review of the PR from Copilot
* Fix zero initialization function (so that the sequence is send - acknowledge - send - acknowledge)
* At the start of the program aks all the motors for the current position

# 05.02
* Moved all the hardware to a perfboard. So that the connection between different components is secure and it helps to reject the idea, that the problem is caused by the loose wire
* Checked the zero initialization. Looked good))).... 

# 06.02
## Results
* Zero initialization stopped working (in both -- my program and by hand). Figured out that the problem is with the 1-st bit of the controlword. 
Problem: The program with zero initialization worked from time to time
After searching for the error, I found out this:
If the controlword has the 1-st (0-index) bit ("output voltage") set, sending EA66 and EA70 to 0x260A does not zero out position (On the contrary, it just sets some strange value to it -- at first sight, I did not find out the relation of this number to the prevoius position). If it is set to 0, zeroing out works

The 1-st bit ("output voltage") is used to enable power to the motor. If
0 -- power is disabled and the motor does not hold the position; If it
is set to 1 -- the motor does hold the position

So the correct zero initialization sequence:
1. Set the 1-st bit of the control word (0x6040) to 0
2. Send EA66 to 0x260A
3. Send EA70 to 0x260A
4. Set the 1-st bit of the control word (0x6040) to 1

* Used USB_CAN TOOL to get acquainted with NMT function. It can be used to reset/reload connection with the motor. 
Problem was with the 3-rd motor. It does not respond to these NMTs (It constantly shows 0x05 -- operational). Could not figure out the problem

And there is also a feature, when the master also  needs to send heartbeat to the nodes. And if it times out, the motors stop their operation. Fun fact: 3-rd motor does respond to master's timeout (it goes to 0x04 -- error)

## What to do tomorrow
* Fix zero initialization sequence (insert the first command, of setting the 1-st bit to 0)
* Clean code (delete unused functions, delete unused files, change numbers into constants with names, set better names to functions and variables, some comments)
* Figure out how to move the motor




## Ideas
* Make, so that heartbeat time is not , when the package is being processed, but when the package was received. But maybe it is also wrong. I will compare that time with the time of processing and will say, that there is timeout, even though there is another heartbeat package only waiting to be processed



# How to structure callback dialogue
Acknowldegement callbacks should have this structure:
1) Set the current callback to either nullptr or to the regular callback (if applicable)
2) Check the response status using checkResponseStatus(). If it fails, return from the function
3) Set the next callback
4) Send the next command
5) Pass status of sending to checkResponseStatus()
6) If sending fails, set the next callback to nullptr or regular callback (if applicable)

Read response callbacks should have this structure:
1) Set the current callback to either nullptr or to the regular callback (if applicable
2) Check the response status using checkResponseStatus(). If it fails, return from the function
3) Process received data
4) Set the next callback
5) Send the next command
6) Pass status of sending to checkResponseStatus()
7) If sending fails, set the next callback to nullptr or regular callback (if applicable)

So read response callbacks differ from acknowldegement callbacks by steps 3 (process data) only.
In the end of the sequence, you can insert a funneling function, which will track if every axis has finished and report the final result