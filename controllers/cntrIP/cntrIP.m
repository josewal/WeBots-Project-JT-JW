%desktop;
TIME_STEP = 32;

IMU = wb_robot_get_device(convertStringsToChars("IMU"));
wb_inertial_unit_enable(IMU, TIME_STEP);

motor_tags = ["wheel1", "wheel2", "wheel3"];
for i = 1:3
    motors(i) = Motor(motor_tags(i));
end


pitchXPID = PID(50,10,5);
pitchXPID.enable();
desired_pitchX = 0;
prev_desired_pitchX = 0;

pitchYPID = PID(50,10,5);
pitchYPID.enable();
desired_pitchY = 0;
prev_desired_pitchY = 0;

sample_setpointX = 0;
sample_positionX = 0;
sample_setpointY = 0;
sample_positionY = 0;

velocity = 1;
speed = [0,0,0];
heading = 0;


fig = figure();
while wb_robot_step(TIME_STEP) ~= -1
    
    
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(IMU)
    
    pitchXPID.update(pitch_roll_yaw(1),0.0005)
    desired_xspeed = pitchXPID.output
    
    pitchYPID.update(pitch_roll_yaw(2),0)
    desired_yspeed = pitchYPID.output
    
    velocity = norm([desired_xspeed,desired_yspeed])
    
    if desired_xspeed > 0;
        heading = asin(desired_yspeed/velocity)+pi;
    else
        heading = asin(desired_yspeed/velocity);
    end
    
    
    speed(1) = velocity * sin(0 - heading);
    speed(2) = velocity * sin(((2*pi)/3) - heading);
    speed(3) = velocity * sin(((4*pi)/3) - heading);
    
    for i = 1:length(motors)
        motors(i).run(speed(i));
    end
    
    
    fig = subplot(2,1,1);
    sample_setpointX = [sample_setpointX, pitchXPID.setpoint];
    sample_positionX = [sample_positionX, pitchXPID.input];
    
    plot3(sample_setpointX, "g-")
    hold on
    plot3(sample_positionX, "b-");
    grid on
    axis([0 inf -0.1 0.1]);
    hold off
    
    sample_setpointY = [sample_setpointY, pitchYPID.setpoint];
    sample_positionY = [sample_positionY, pitchYPID.input];
    
    
    fig = subplot(2,1,2);
    plot3(sample_setpointY, "g-")
    hold on
    plot3(sample_positionY, "b-");
    grid on
    axis([0 inf -0.1 0.1]);
    hold off
    

drawnow;
end