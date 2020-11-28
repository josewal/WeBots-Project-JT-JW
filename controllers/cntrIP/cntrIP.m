%desktop;

TIME_STEP = 16;

sensor = wb_robot_get_device(convertStringsToChars("IMU"));
wb_inertial_unit_enable(sensor, TIME_STEP);

receiver = wb_robot_get_device(convertStringsToChars("receiver"));
wb_receiver_enable(receiver, TIME_STEP)


motor_tags = ["wheel1", "wheel2", "wheel3", "wheel4"];
for i = 1:4
    motors(i) = Motor(motor_tags(i));
end


speedPID = PID(0.05, 0.001,0);
speedPID.setLimits(-0.1, 0.1);
speedPID.disable();


pitchPID = PID(400, 40, 0.4);
desired_pitch = 0;

sample_error = 0;
sample_setpoint = 0;

msg = [];
distance = 0;

t = 0;


while wb_robot_step(TIME_STEP) ~= -1
    t = t + 1;
    while wb_receiver_get_queue_length(receiver) > 0
        pointer = wb_receiver_get_data(receiver);
        setdatatype(pointer, 'doublePtr', 1, 1);
        msg = get(pointer, 'Value');
        wb_receiver_next_packet(receiver);
    end
    
    velocity = msg;
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(sensor);
    pitch = pitch_roll_yaw(1);
    
    %speedPID.update(velocity, 0);
    %desired_pitch = speedPID.output;
    
    if t > 50
    desired_pitch = 0.05;
    end
    
    pitchPID.update(pitch, desired_pitch);
    desired_motor_speed = pitchPID.output;
    
 
    if t < 200
    sample_error = [sample_error, pitch];
    sample_setpoint = [sample_setpoint, desired_pitch];
    
    plot(sample_setpoint, "g-")
    hold on
    plot(sample_error, "b-");
    grid on
    axis([40 inf -0.025 inf]);
    hold off
   
    end
    
    
    for i = 1:length(motors)
        motors(i).run(-desired_motor_speed);
    end
    
    drawnow;
end