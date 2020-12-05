%desktop;
TIME_STEP = 32;

IMU = wb_robot_get_device(convertStringsToChars("IMU"));
wb_inertial_unit_enable(IMU, TIME_STEP);

motor_tags = ["arm motor", "wheel"];
for i = 1:length(motor_tags);
    motors(i) = Motor(motor_tags(i));
end




velocityPID = PID(0.005, 0.01, 0.1);
velocityPID.enable();
velocityPID.setLimits(-1,1);
desired_velocity = 0;
velocity = [0,0];


wheelPID = PID(50, 50, 10);
wheelPID.enable();
wheelPID.setLimits(-50,50);
desired_pitchY = 0;


pitchXPID = PID(100, 100, 100);
pitchXPID.enable();
pitchXPID.setLimits(-50,50);
desired_pitchX = 0;

sample_setpointX = 0;
sample_positionX = 0;

t = 0;
speeds = [0,0];

while wb_robot_step(TIME_STEP) ~= -1
    
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    
    velocity = [velocity(2), velocity(2) * 0 ...
                             + 1 * (speeds(2)*0.05)];
    velocityPID.update(velocity(end), desired_velocity);
    desired_pitchY = -velocityPID.output;
    
    
    wheelPID.update(pitch_roll_yaw(1), desired_pitchY);
    speeds(2) = wheelPID.output;

    
    pitchXPID.update(pitch_roll_yaw(2),desired_pitchX);
    speeds(1) = -pitchXPID.output;
    

    for i = 1:length(motors)
        motors(i).run(speeds(i));
    end
    
    if t(end) == 100
        desired_pitchX = 0.05;
    elseif t(end) == 120
        desired_pitchX = 0
    elseif t(end) == 250
      desired_pitchX = -0.05;
    elseif t(end) == 280
       desired_pitchX = 0;
       end
      

    sample_setpointX = [sample_setpointX(end), pitchXPID.setpoint(end)];
    sample_positionX = [sample_positionX(end), pitchXPID.input];
    
    
    t = [t(end), t(end) + 1];
    

    hold on
    plot(t, sample_setpointX, "b-");
    plot(t, sample_positionX, "r-");
    axis([0 inf -0.2 0.2]);
    

drawnow;
end