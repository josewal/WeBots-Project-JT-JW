%desktop;
TIME_STEP = 32;

IMU = wb_robot_get_device(convertStringsToChars("IMU"));
wb_inertial_unit_enable(IMU, TIME_STEP);

acc = wb_robot_get_device(convertStringsToChars("accelerometer"));
wb_accelerometer_enable(acc, TIME_STEP);

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


rollPID = PID(700, 100, 700);
rollPID.enable();
rollPID.setLimits(-100,100)
desired_roll = 0;


yawPID = PID(5, 0.25, 1);
yawPID.enable();
yawPID.setLimits(-50,50)
desired_yaw = 0;

bankPID = PID(0.1, 0.125, 0.02);
bankPID.enable();
bankPID.setLimits(-0.2,0.2)
desired_bank = 0;

sample_setpoint = 0;
sample_position = 0;
sample_output = 0;

t = 0;
speeds = [0,0,0];
balance_angle_LF = [0,0];

yaw = 0;
prev_yaw = 0;
pitch_roll_yaw = [0,0,0];
prev_pitch_roll_yaw = [0,0,0];
rotations = 0;
hold on
while wb_robot_step(TIME_STEP) ~= -1
    t = [t(end), t(end) + 1];
    
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  
    d = prev_pitch_roll_yaw(3) - pitch_roll_yaw(3);

    if d > pi
      rotations = rotations + 2*pi;
      plot(t(end), rotations, "b*");
    elseif d < -pi
      rotations = rotations - 2*pi;
      plot(t(end), rotations, "b*");
    end
  
    prev_yaw = yaw;
    yaw = pitch_roll_yaw(3) + rotations;
    prev_pitch_roll_yaw = pitch_roll_yaw;
    pitch_roll_yaw(3) = yaw;
  
    plot(t, [prev_yaw, yaw], "r-");
    plot(t(end), d, "g*");
    
    
    
    acc_x_y_z = wb_accelerometer_get_values(acc);
    balance_vector = [acc_x_y_z(1), -acc_x_y_z(3)];
    
    
    if balance_vector(1) > 0
    balance_angle = +(balance_vector(1)/norm(balance_vector));
    elseif balance_vector(1) < 0
    balance_angle = +(balance_vector(1)/norm(balance_vector));
    else 
    balance_angle = 0;
    end
    
    balance_angle_LF = [balance_angle_LF(end),...
                        0.9*balance_angle_LF(end) + 0.1*balance_angle];
    
    
    if speeds(1) > 20
    desired_bank = -0.05;
    elseif speeds(1) < -20
    desired_bank = 0.05;
    else
    desired_bank = 0
    end
    
    
    if t(end) > 20            
    bankPID.update(-balance_angle_LF(end), desired_bank);
    desired_roll = bankPID.output;
    else
    desired_roll = 0;
    end
    
    velocity = speeds(2) * 0.05;
    velocityPID.update(velocity(end), desired_velocity);
    desired_pitch = -velocityPID.output;
    
    pitchPID.update(pitch_roll_yaw(1), desired_pitch);
    speeds(2) = pitchPID.output;
    
    rollPID.update(pitch_roll_yaw(2),desired_roll);
    speeds(1) = -rollPID.output
    
    
    if t(end) > 30
    desired_yaw = desired_yaw - 0.025;
    end
    
    yawPID.update(pitch_roll_yaw(3),desired_yaw);
    speeds(3) = yawPID.output;
    


    for i = 1:length(motors)
        motors(i).run(speeds(i));
    end
    
    
      

    %sample_setpoint = [sample_setpoint(end), bankPID.setpoint(end)];
    %sample_position = [sample_position(end), bankPID.input];
    %sample_output = [sample_output(end), bankPID.output];
    
    
    
    
    %subplot(1,2,1);
    %plot([0,balance_vector(1)], [0,balance_vector(2)], "-r");
    %axis ([-10, 10, -10, 10])
    
    %subplot(1,2,2);
    %hold on
    %plot(t, sample_setpoint, "b-");
    %plot(t, sample_position, "r-");
    %plot(t, sample_output, "g-");
    %axis([t(end)-100, t(end), -inf, inf]);
    

drawnow;
end