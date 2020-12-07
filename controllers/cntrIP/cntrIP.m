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
desired_velocity = 1;
velocity = [0,0];


pitchPID = PID(50, 50, 10);
pitchPID.enable();
pitchPID.setLimits(-50,50);
desired_pitch = 0;


rollPID = PID(700, 100, 2250);
rollPID.enable();
rollPID.setLimits(-100,100)
desired_roll = 0;


yawPID = PID(5, 0, 1);
yawPID.enable();
yawPID.setLimits(-50,50)
desired_yaw = 0;

sample_setpoint = 0;
sample_position = 0;

t = 0;
speeds = [0,0,0];

while wb_robot_step(TIME_STEP) ~= -1
    
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    
    velocity = [velocity(2), velocity(2) * 0 ...
                             + 1 * (speeds(2)*0.05)];
    velocityPID.update(velocity(end), desired_velocity);
    desired_pitch = -velocityPID.output;
    
    if t(end) > 50
    desired_yaw = -2;
    end

    
    
    
    pitchPID.update(pitch_roll_yaw(1), desired_pitch);
    speeds(2) = pitchPID.output;
    
    %if  abs(yawPID.e(end)) > 0.2
    %desired_roll = -0.5*yawPID.e(end);
    %end
    
    rollPID.update(pitch_roll_yaw(2),desired_roll);
    speeds(1) = -rollPID.output;
    
    yawPID.update(pitch_roll_yaw(3),desired_yaw);
    speeds(3) = yawPID.output;
    


    for i = 1:length(motors)
        motors(i).run(speeds(i));
    end
    
    
      

    %sample_setpoint = [sample_setpoint(end), rollPID.setpoint(end)];
    %sample_position = [sample_position(end), rollPID.input];
    
    
    t = [t(end), t(end) + 1];
    

    %hold on
    %plot(t, sample_setpoint, "b-");
    %plot(t, sample_position, "r-");
    %axis([t(end)-100, t(end), -inf, inf]);
    

drawnow;
end