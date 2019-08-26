# Rocket-Data-Logger
Collects telemetry from a model rocket and transmits to a ground station using RF69 long-distance radio.

# Ingredients: 
* two Adafruit Feather M0's with RF95 long-range radio https://www.adafruit.com/product/3178
* one BMP180 pressure sensor https://www.adafruit.com/product/1603
* one ADXL377 high-g accelerometer https://www.adafruit.com/product/1413
* one micro-SD card breakout board https://www.adafruit.com/product/254
* one Estes Loadstar model rocket https://estesrockets.com/product/003227-loadstar-ii/
* one small LiPo battery https://www.adafruit.com/product/1570

# Transmitter
I was able to piggyback the pressure sensor and accelerometer directly onto the Feather board, creating a compact package that mounts into
the rocket's payload bay using foam blocks for support.  I used standard 1/4 wave length of solid-core wire for the antenna, protruding through into the nose cone.
Battery lifetime was greater than 24 hours.

# Receiver
Receiver logs data simultaneously to serial output and to an SD card.  This receiver is generic: it just listens to the transmitter and 
records everything it hears, so it can be used for other projects sending different telemetry.
