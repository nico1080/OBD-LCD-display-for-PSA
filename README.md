# OBD-LCD-display-for-PSA
LCD display to show vehicle data on PSA car


Display various data from OBD port on LCD (you need correct adress from diagbox, see mine in github https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/adress-database.ods)

- Various screens are available
- LCD backlight brightness is adjusted depending of sate of car  lightening (day/night/black panel mode). Each screens can have specific value
- To change screen you need to press the "Return" button on wheel (bottom right)
- At startup it display the init (0) screen for 10s (typre pressure oil level etc)
- Then it display the temprature (1) screen
- If you do a short press on "Return) it will switch to screen (2) (air flow) then 3 (battery) then 4 (power) and go back to 0 (init)


I also implemented "screen sequence " but I don't realy use it
- if you do a long press you will go to screen 100 (sequence 0-->1) a short press will make you change screen in this sequence 100-->101-->100
- Same thing for sequence 2 (screen 200 & 201)
- A last long press will make you go back seq 0

For Hardware you will need:
- Arduino (I used nano)
- MCP2515 can module
- TFT_ILI9341 3.2 LCD display
- OBD 16 pin connector
- 5V power supply


I also 3D printed a case for the electronic but my design is not perfect

In car with Init screen:
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/view.jpg)

Init screen with tyre pressure/temperature, oil level battery state etc
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen0.jpg)

Temperature screen (1)  (white values are engine setpoint)
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen1.jpg)

Air screen (2) (white values are engine setpoint)
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen2.jpg)

Battery screen (3) (need some work ! )
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen3.jpg)

Fuel / power screen (4)
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen4.jpg)
