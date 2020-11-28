%desktop;

TIME_STEP = 32;

sensor = wb_robot_get_device(convertStringsToChars("IMU"));
wb_inertial_unit_enable(sensor, TIME_STEP);

receiver = wb_robot_get_device(convertStringsToChars("receiver"));
wb_receiver_enable(receiver, TIME_STEP)


motor_tags = ["wheel1", "wheel2", "wheel3", "wheel4"];
for i = 1:4
    motors(i) = Motor(motor_tags(i));
end


speedPID = PID(0.3, 0.00005, 0.004);
speedPID.setLimits(-0.1, 0.1);
speedPID.enable();
velocity = 0;
desired_velocity = 0;


pitchPID = PID(120,15,0);
pitchPID.enable();
desired_pitch = 0;
prev_desired_pitch = 0;

sample_errorP = 0;
sample_setpointP = 0;
sample_errorV = 0;
sample_setpointV = 0;

msg = [];
distance = 0;

t = 0;

fig = figure();
while wb_robot_step(TIME_STEP) ~= -1
    t = t + 1;
    while wb_receiver_get_queue_length(receiver) > 0
        pointer = wb_receiver_get_data(receiver);
        setdatatype(pointer, 'doublePtr', 1, 1);
        msg = get(pointer, 'Value');
        wb_receiver_next_packet(receiver);
    end
    
    if size(msg) == [1,1]
    velocity = msg;
    end
    
    
    
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(sensor);
    pitch = pitch_roll_yaw(1);
    
    if t == 50 
    desired_velocity = 1;
    elseif t == 200
    desired_velocity = 0.0;
    elseif t == 400
    desired_velocity = -0.5;
    end
    
    speedPID.update(velocity, desired_velocity);
    
    
    desired_pitch = 0.9*prev_desired_pitch + 0.1*speedPID.output;
    prev_desired_pitch = desired_pitch;
 
    pitchPID.update(pitch, desired_pitch);
    desired_motor_speed = pitchPID.output;
 
    if (t < 500)
    fig = subplot(2,1,1);
    sample_errorP = [sample_errorP, pitch];
    sample_setpointP = [sample_setpointP, desired_pitch];
    
    plot(sample_setpointP, "g-")
    hold on
    plot(sample_errorP, "b-");
    grid on
    axis([0 inf -inf inf]);
    hold off
    
     sample_errorV = [sample_errorV, velocity];
    sample_setpointV = [sample_setpointV, desired_velocity];
    
    fig = subplot(2,1,2);
    plot(sample_setpointV, "g-")
    hold on
    plot(sample_errorV, "b-");
    grid on
    axis([0 inf -inf inf]);
    hold off
   end
    
    
    for i = 1:length(motors)
        motors(i).run(-desired_motor_speed);
    end
    
    drawnow;
end