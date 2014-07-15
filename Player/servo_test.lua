module(... or '', package.seeall)

-- Get Platform for package path
cwd = '.';
local platform = os.getenv('PLATFORM') or '';
if (string.find(platform,'webots')) then cwd = cwd .. '/Player';
end

-- Get Computer for Lib suffix
local computer = os.getenv('COMPUTER') or '';
if (string.find(computer, 'Darwin')) then
  -- MacOS X uses .dylib:
  package.cpath = cwd .. '/Lib/?.dylib;' .. package.cpath;
else
  package.cpath = cwd .. '/Lib/?.so;' .. package.cpath;
end

package.path = cwd .. '/?.lua;' .. package.path;
package.path = cwd .. '/Util/?.lua;' .. package.path;
package.path = cwd .. '/Config/?.lua;' .. package.path;
package.path = cwd .. '/Lib/?.lua;' .. package.path;
package.path = cwd .. '/Dev/?.lua;' .. package.path;
package.path = cwd .. '/Motion/?.lua;' .. package.path;
package.path = cwd .. '/Motion/keyframes/?.lua;' .. package.path;
package.path = cwd .. '/Motion/Walk/?.lua;' .. package.path;
package.path = cwd .. '/Vision/?.lua;' .. package.path;
package.path = cwd .. '/World/?.lua;' .. package.path;

require('DynamixelPacket');
require('Dynamixel');
require('unix');
require('shm');
require('carray');
require('vector');
require('Config');
require('Transform');
require('getch');

print("Robot ID:",Config.game.robotID);

dirReverse = Config.servo.dirReverse;
posZero=Config.servo.posZero;
gyrZero=Config.gyro.zero;
legBias=Config.walk.servoBias;
armBias=Config.servo.armBias;
idMap = Config.servo.idMap;
nJoint = #idMap;
scale={};
for i=1,nJoint do 
  scale[i]=Config.servo.steps[i]/Config.servo.moveRange[i];
end

tLast=0;
count=1;
battery_warning=0;
battery_led1 = 0;
battery_led2 = 0;
battery_blink = 0;

chk_servo_no=0;
nButton = 0;


print("nJoint"..nJoint);
getch.enableblock(1);

function sync_command()
  local addr = 30;
  local ids = {};
  local data = {};
  local n = 0;
  for i = 1,#idMap do
   
      n = n+1;
      ids[n] = idMap[i];
      local word=0;
      word = command[i]+posZero[i];
      
      data[n] = math.min(math.max(word, 0), Config.servo.steps[i]-1);
   
  end
  if (n > 0) then
    Dynamixel.sync_write_word(ids, addr, data);
  end
end

function sync_hardness()
  local addr=34; --hardness is working with RX28
  local ids = {};
  local data = {};
  local n = 0;
  for i = 1,#idMap do
    n = n+1;
    ids[n] = idMap[i];
    data[n] = 1023*hardness[i];
  end
  if (n > 0) then
    Dynamixel.sync_write_word(ids, addr, data);
  end
end

function torque_enable()
  local addr = 24;
  local ids = {};
  local data = {};
  local n = 0;
  for i = 1,#idMap do
    n = n+1;
    ids[n] = idMap[i];
    data[n] = torqueEnable;
  end
  if (n > 0) then
    Dynamixel.sync_write_byte(ids, addr, data);
	  print("Torque enable changed !!!");
  end   

end


print("nJoint"..nJoint);
command = vector.zeros(nJoint);
torqueEnable = 1;


focusIndex = 1;

function process_keyinput()
  local str=getch.get();
  local c =0;

  if #str> 0 then
    
    local byte=string.byte(str,1);

    if byte==string.byte("w") then  focusIndex = focusIndex -1 ;
    elseif byte==string.byte("s") then  focusIndex = focusIndex +1 ;
    elseif byte==string.byte("a") then  command[focusIndex] = command[focusIndex] -1;
    elseif byte==string.byte("q") then  command[focusIndex] = command[focusIndex] -5;
    elseif byte==string.byte("d") then  command[focusIndex] = command[focusIndex] +1;
    elseif byte==string.byte("e") then  command[focusIndex] = command[focusIndex] +5;
    
	end

    if focusIndex < 1 then focusIndex = 1; 
    elseif focusIndex > nJoint then focusIndex = nJoint;
    end

    c = c+1;
  end

  return c;
end




hardness = vector.ones(nJoint)*0.3;
-- main prog

Dynamixel.open();
unix.usleep(200000);
twait = 0.020;
Dynamixel.ping_probe();

--enable torque and set hardness

torque_enable();
sync_hardness();
sync_command();


--print(actuator.command[0]);

while 1 do

  local c = process_keyinput() ;

  if c > 0 then
    -- os.execute("clear");
    -- print sth
    
    torque_enable();
    sync_command();

    --[[
    {
      512,512, --Head
      512,512,512, --LArm
      2048,2048,2048,2048,2048,2048, --LLeg
      2048,2048,2048,2048,2048,2048, --RLeg
      512,512,512, --RArm
      --    512,    --For aux
    }
    --]]
          os.execute("clear")
    print("index"..focusIndex);
    
--    print(string.format("hheoo %d",actuator.command[0]));
    print(string.format("{%d,%d,\n %d,%d,%d,\n%d,%d,%d,%d,%d,%d,\n%d,%d,%d,%d,%d,%d,\n%d,%d,%d}\n", unpack(command)) );

  end


  unix.usleep(1000);
end



