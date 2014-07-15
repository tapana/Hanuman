# Nothing to compile for NaoBody

all:: dynamixel imu

dynamixel::
	cd Dynamixel && make && cd $(CWD)

imu::
	cd IMU && make && cd $(CWD)

clean:
	cd Dynamixel && make clean && cd $(CWD)
	cd IMU && make clean && cd $(CWD)

