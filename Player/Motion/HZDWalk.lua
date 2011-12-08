module(..., package.seeall);

require('Body')
require('Kinematics')
require('Config');
require('Config_OP_HZD')
require('vector')
require 'util'

t0 = Body.get_time();

-- Walk Parameters
hardnessLeg_gnd = Config.walk.hardnessLeg;
hardnessLeg_gnd[5] = 0; -- Ankle pitch is free moving
hardnessLeg_air = Config.walk.hardnessLeg;

-- For Debugging
saveCount = 0;
jointNames = {"Left_Hip_Yaw", "Left_Hip_Roll", "Left_Hip_Pitch", "Left_Knee_Pitch", "Left_Ankle_Pitch", "Left_Ankle_Roll", "Right_Hip_Yaw", "Right_Hip_Roll", "Right_Hip_Pitch", "Right_Knee_Pitch", "Right_Ankle_Roll", "Right_Ankle_Roll"};
logfile_name = string.format("/tmp/joint_angles.raw");


function update( supportLeg )
  
  if( supportLeg == 0 ) then -- Left left on ground
    Body.set_lleg_hardness(hardnessLeg_gnd);
    Body.set_rleg_hardness(hardnessLeg_air);    
    stance_leg = Body.get_lleg_position();
    alpha = Config_OP_HZD.alpha_L;
  else
    Body.set_rleg_hardness(hardnessLeg_gnd);
    Body.set_lleg_hardness(hardnessLeg_air);    
    stance_leg = Body.get_rleg_position();
    alpha = Config_OP_HZD.alpha_R;
  end
  
  t = Body.get_time();
  
  theta = stance_leg[5]; -- Just use the ankle
  theta_min = 0.01294;
  theta_max = -0.3054;
  s = (theta - theta_min) / (theta_max - theta_min);
  
  qLegs = vector.zeros(12);
  for i=1,12 do
    qLegs[i] = util.polyval_bz(alpha[i], s);
  end

  Body.set_lleg_command(qLegs);


  -- Debug Printing in degrees
  for i=1,12 do
    print( jointNames[i] .. ':\t'..qLegs[i]*180/math.pi );
  end
  print();

end

function record_joint_angles()

  -- Open the file
  local f = io.open(logfile_name, "a");
  assert(f, "Could not open save image file");
  if( saveCount == 0 ) then
    -- Write the Header
    f:write( "time,Left,Right,IMU_Roll,IMU_Pitch,IMU_Yaw" );
    for i=1,12 do
      f:write( string.format(",%s",jointNames[i]) );
    end
    f:write( "\n" );
  end

  -- Write the data
  local t = Body.get_time();
  f:write( string.format("%f",t-t0) );
  local imuAngle = Body.get_sensor_imuAngle();
  f:write( string.format(",%f,%f,%f",unpack(imuAngle)) );
  local lleg = Body.get_lleg_position();
  for i=1,6 do
    f:write( string.format(",%f",lleg[i]) );
  end
  local rleg = Body.get_rleg_position();
  for i=1,6 do
    f:write( string.format(",%f",rleg[i]) );
  end
  f:write( "\n" );
  -- Close the file
  f:close();
  saveCount = saveCount + 1;

end

