# OBD-LCD-display-for-PSA
LCD display to show vehicle data on PSA car


Display various data from OBD port on LCD (you need correct adress from diagbox)

- Various screens are available
- LCD backlight brightness is adjusted depending on sate of car  lightening (day/night/black panel mode). Each screen can have specific value
- To change screen you need to press the "Return" button on wheel (bottom right)
- At startup it display the init (0) screen for 10s (typre pressure oil level etc)
- Then it display the temprature (1) screen
- If you do a short press on "Return) it will switch to screen (2) (air flow) then 3 (battery) then 4 (power) and go back to 0 (init)


I also implemented "screen sequence " but I don't realy use it
- if you do a long press you will go to screen 100 (sequence 0-->1) a short press will make you change screen in this sequence 100-->101-->100
- Same thing for sequence 2 (screen 200 & 201)
- A last long press will make you go back seq 0

![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/view.jpg)

![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen0.jpg)
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen1.jpg)
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen2.jpg)
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen3.jpg)
![alt text](https://github.com/nico1080/OBD-LCD-display-for-PSA/blob/main/picture/screen/screen4.jpg)
