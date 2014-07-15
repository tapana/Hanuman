package.cpath = '/home/ivy/project/humanoid/darwin-op/UPennalizers/Player/Lib/?.so;' .. package.cpath;

require('Dynamixel');

twait = 0.010;

Dynamixel.open();
Dynamixel.ping_probe();
