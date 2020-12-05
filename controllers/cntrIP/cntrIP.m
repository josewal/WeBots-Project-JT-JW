%desktop;
TIME_STEP = 32;

IMU = wb_robot_get_device(convertStringsToChars("IMU"));
wb_inertial_unit_enable(IMU, TIME_STEP);

motor_tags = ["arm motor", "wheel", "yaw motor"];
for i = 1:length(motor_tags);
    motors(i) = Motor(motor_tags(i));
end




velocityPID = PID(0.02, 0, 0.01);
velocityPID.enable();
velocityPID.setLimits(-1,1);
desired_velocity = 0.5;
velocity = [0,0];


wheelPID = PID(50, 50, 10);
wheelPID.enable();
wheelPID.setLimits(-50,50);
desired_pitchY = 0;


pitchXPID = PID(700, 100, 2250);
pitchXPID.enable();
pitchXPID.setLimits(-100,100)
desired_pitchX = 0;

yawPID = PID(1, 0, 1);
yawPID.enable();
yawPID.setLimits(-50,50)
desired_yaw = 0;

sample_setpointX = 0;
sample_positionX = 0;

t = 0;
speeds = [0,0,0];

while wb_robot_step(TIME_STEP) ~= -1
    
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    
    velocity = [velocity(2), velocity(2) * 0 ...
                             + 1 * (speeds(2)*0.05)];
    velocityPID.update(velocity(end), desired_velocity);
    desired_pitchY = -velocityPID.output;
    
    
    wheelPID.update(pitch_roll_yaw(1), desired_pitchY);
    speeds(2) = wheelPID.output;

     if abs(yawPID.e(end)) > 5
      desired_pitchX = yawPID.e(end)*0.1;
     else 
       desired_pitchX = 0;
     end
     
    pitchXPID.update(pitch_roll_yaw(2),desired_pitchX);
    speeds(1) = -pitchXPID.output;
    
    if (yawPID.input < 1) && (t(end ) < 500)
    desired_yaw = desired_yaw + 0.01;
    elseif (yawPID.input > -2) && (t(end )> 1000)
    desired_yaw = desired_yaw -0.01;
    end
    
    yawPID.update(pitch_roll_yaw(3),desired_yaw);
    speeds(3) = speeds(3) + yawPID.output;
    

    for i = 1:length(motors)
        motors(i).run(speeds(i));
    end
    
    
      

    %sample_setpointX = [sample_setpointX(end), yawPID.setpoint(end)];
    %sample_positionX = [sample_positionX(end), yawPID.input];
    
    
    t = [t(end), t(end) + 1];
    

    %hold on
    %plot(t, sample_setpointX, "b-");
    %plot(t, sample_positionX, "r-");
    %axis([t(end)-100, t(end), -inf, inf]);
    

drawnow;
end