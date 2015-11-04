tron: tron.cpp
	echo "Enable Motor Pins:7, 11, 22"
	cd /home/pi/piBits/ServoBlaster/user
	sudo ./servod --p1pins = 7,11,22
	cd /home/pi/R.\ PIE/
	g++ tron.cpp -l wiringPi -o tron

run: tron
	sudo ./tron

make stop:
	sudo killall servod
