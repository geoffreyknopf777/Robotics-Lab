tron: tron.cpp drive.h sonar.h
	g++ tron.cpp  -l wiringPi -o tron

enable:
	echo "Enable Motor Pins:7, 11, 12"
	sudo /home/pi/PiBits/ServoBlaster/user/servod --p1pins=7,11,12

run: tron
	sudo ./tron

make stop:
	sudo killall servod
