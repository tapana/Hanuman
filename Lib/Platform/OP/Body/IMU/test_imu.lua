require('unix')
require('IMU')

IMU.init();
count =0;
s = 180.0/3.14159265359;
print(s);


while 1 do

  IMU.update();
  e = IMU.get_euler();
  r = IMU.get_raw();


  if count >=10 then
	count =0;	
	--print(string.format("imu %f %f %f", e[1]*s,e[2]*s,e[3]*s));

	print(string.format("imu %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f", r[1],r[2],r[3], r[4],r[5],r[6] ));
  end
  count = count+1;
  unix.usleep(10000);

end


