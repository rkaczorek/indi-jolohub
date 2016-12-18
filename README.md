# indi-astrohub
Astrohub driver provides support for [AstroHub 3.0](http://astrojolo.blogspot.com/p/astrohub-30.html), which can handle:
- 2x stepper motors Robofocus compatible
- 1x DC motor Skywatcher motofocus compatible
- 4x regulated PWMs
- 4x 12V switchable power lines
- 2x DHT22 temperature and humidity sensors
- 1x DS1820 temperature sensor

Additionally Astrohub 3.0 provides support for:
- Hand controler for manual focus control
- 5V power line
- regulated power line eg. for powering your DSLR
- COMM expansion slot with access to Arduino Serial2 and I2C and 5V power line, which supports addon modules such as:
  - MLX sky temperature sensor
  - TSL237 skyglow sensor
  - Atmospheric pressure sensor
  - GPS module
  - Gyro and compass module
  - Relays module
  - WLAN/BT wireless module

# How to start?
First, you need to download and install INDI server and libraries. See [INDI site](http://indilib.org/download.html) for more details.
In most cases it's enough to run:
```
sudo apt-add-repository ppa:mutlaqja/ppa
sudo apt-get update
sudo apt-get install libindi1
```
Second, download and install indi-astrohub.

Compiling from source:
```
sudo apt-get install indilib-dev
git clone https://github.com/rkaczorek/indi-astrohub.git
cd indi-astrohub
mkdir build
cd build
cmake ..
make
make install
```
Installing from binaries:
```
wget https://github.com/rkaczorek/indi-astrohub/raw/master/binaries/indi-astrohub-latest.deb
dpkg -i indi-astrohub-latest.deb
```

#How to use it?
Start your INDI server with Astrohub drivers:

`indiserver -l /var/log/indi -f /var/run/indi -p 7624 indi_astrohub`

Start KStars with Ekos, connect to your INDI server and enjoy!
