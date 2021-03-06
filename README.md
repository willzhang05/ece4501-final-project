# Final-Project

# Files
Slides are in the file `slides.pdf`.
The report PDF and source are in the `report` directory.
The demo is here: https://www.youtube.com/watch?v=OmV5wOkYNA8.
Here are more demo videos (we forgot to push the restart button in the above demo!):
* https://www.youtube.com/watch?v=TUwJ-5sQj-w
* https://www.youtube.com/watch?v=lhYUkJOL8_g

## ATTENTION!
Our mini-project requires additional steps to work!
This is due to our use of EEPROM storage.

# Steps
To run our code, there are a few additional steps. 

* Open the project file `driverlib/driverlib.uvproj`

* Build all targets for this file. 

* This should create a `.lib` file in `driverlib/rvmdk/`

* Open the Keil project file `finalproj.uvproj`

* In Keil, right click on the source folder and select 'Add existing files.' 

* In the dropdown, select `.lib` as the file type

* Navigate to `driverlib/rvmdk` and select the `.lib` file

* Build target and run!

# If EEPROM does not work!

Find and delete the following line at the top of `Main.c`.

```c
#define USE_NV_LEADERBOARD
```
