# 26.01
## Results
- Moved initialisation to setup()
- Moved can management into CanOpen
- Renamed MyCanOpen to CanOpen
- Resolved an algorithm of how to produce features in the future (alogrithms stays the same)
- Pushed to github and gitlab changes. Created and merged pull requests
- Loopback test before CAN initialisation

## What to do tomorrow (draft)
- Delete unused functions in the repo
- Clean "Robot" folder
- Change/Check, that CanOpen is used to implement functions like "set speed", "set target position", ...
- Using Perplexity define an algorithm for zero initialisation of motors. So that the motors think, that current position is 0
- Move Axis array somewhere (probably in MoveController), so that I do not explicitely have an array. Maybe use std::array
- Set constants to the same format
- Check is I can switch Serial2 to Serial
- 
