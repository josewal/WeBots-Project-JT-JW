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

setpoint = 0.5;
speedPID = PID(0.01, 0.001,0);
speedPID.setLimits(-0.1, 0.1);
speedPID.enable();

pitchPID = PID(100, 7, 0.5);

msg = [];
while wb_robot_step(TIME_STEP) ~= -1
    
    while wb_receiver_get_queue_length(receiver) > 0
        pointer = wb_receiver_get_data(receiver);
        setdatatype(pointer, 'doublePtr', 1, 1);
        msg = get(pointer, 'Value');
        wb_receiver_next_packet(receiver);
    end
    
    velocity = msg;
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(sensor);
    pitch = pitch_roll_yaw(1);
    
    speedPID.update(velocity, setpoint);
    desired_pitch = speedPID.output;
    
    pitchPID.update(pitch, desired_pitch);
    desired_motor_speed = pitchPID.output;
    
    
    for i = 1:length(motors)
        motors(i).run(-desired_motor_speed);
    end
    
    drawnow;
end