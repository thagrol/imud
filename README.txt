imud.py

Python daemon to share data from an IMU (inertial measurement unit) with multiple client processes via TCP/IP.

Hardware Requirements:
	Raspberry Pi
	i2c connected IMU using LSM9DS0 or compatible
	i2c connected BMP180 or compatible
	Note: sensors are expected to be mounted parallel to the Pi board.

Software Requirements:
	configured and functioning i2c stack
	python 2.7.3 or later (may work with earlier versions, not yet tested with python 3)
	RPi.GPIO python module

Installation:
	Download imud.py.
	Copy to a location of your choice.
	Start with:
		sudo /path/to/imud.py

Command Line Options:
usage: imud.py [-h] [-p PORT] [-l | -b W.X.Y.Z] [-i {0,1}] [-g {0x6a,0x6b}]
               [-m {0x1d,0x1e}] [-n] [--debug]

optional arguments:
  -h, --help            show this help message and exit
  -p PORT, --port PORT  port to listen on. (default: 7691)
  -l, --local           bind to localhost only.
  -b W.X.Y.Z, --bind W.X.Y.Z
                        address of interface to bind to.
  -i {0,1}, --i2c {0,1}
                        i2c bus to use.(default: 1)
  -g {0x6a,0x6b}, --gyro {0x6a,0x6b}
                        address of gyroscope (default: 0x6a)
  -m {0x1d,0x1e}, --motion {0x1d,0x1e}
                        address of magnetometer and accelerometer (default:
                        0x1e)
  -n, --nodaemon        do not daemonize. Run in foreground instead.
  --debug               enabled debug output

If neither -l nor -b are present all available interfaces are bound.


Clients:

All communication is in ASCII plain text and human readable. Sample clients can be found in the “clients” directory.

The simplest way to check the server is working is as follows:
1. Open a connection to the desired server and port e.g.
	telnet localhost 7691
2. Check the incoming text for content etc.
3. Close connection.

See protocol.txt for full information.

