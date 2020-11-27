TIME_STEP = 16;

sensor = wb_robot_get_device(convertStringsToChars("IMU"));
wb_inertial_unit_enable(sensor, TIME_STEP);

receiver = wb_robot_get_device(convertStringsToChars("receiver"));
wb_receiver_enable(receiver, TIME_STEP)

wheels = [];
wheels_names = [ "wheel1", "wheel2", "wheel3", "wheel4" ];
for i = 1:4
  wheels(i) = wb_robot_get_device(convertStringsToChars(wheels_names(i)));
  wb_motor_set_position(wheels(i), inf);
  wb_motor_set_velocity(wheels(i), 0.0);
end

setpoint = 0.5;
speedPID = PID(0.001, 0.001,0);
speedPID.setLimits(-0.1, 0.1);
pitchPID = PID(30, 7, 0.5);



msg = [];
while wb_robot_step(TIME_STEP) ~= -1

  while wb_receiver_get_queue_length(receiver) > 0
  pointer = wb_receiver_get_data(receiver);
  setdatatype(pointer, 'doublePtr', 1, 1);
  msg = get(pointer, 'Value');
  wb_receiver_next_packet(receiver);
  end
  
  real_speed = msg;
  pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(sensor);
  pitch = pitch_roll_yaw(1);
 
  speedPID.update(real_speed, setpoint);
  speedPID.compute();
  
  pitchPID.update(pitch, speedPID.output);
  pitchPID.compute();
  
  motor_speed = -pitchPID.output; 
  
  wb_console_print(sprintf('Real speed: %f\n', real_speed), WB_STDOUT);
  
  
  wb_motor_set_velocity(wheels(1), motor_speed);
  wb_motor_set_velocity(wheels(2), motor_speed);
  wb_motor_set_velocity(wheels(3), motor_speed);
  wb_motor_set_velocity(wheels(4), motor_speed);
  drawnow;
end