desktop

TIME_STEP = 32;

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

theta = [];
real_speed = 0;

pitch = PID(50, 10, 0);
spPID = PID(1, 0, 0);

msg = [];
while wb_robot_step(TIME_STEP) ~= -1

  while wb_receiver_get_queue_length(receiver) > 0
  pointer = wb_receiver_get_data(receiver);
  setdatatype(pointer, 'doublePtr', 1, 1);
  msg = get(pointer, 'Value');
  wb_receiver_next_packet(receiver);
  end
  
  real_speed = msg;
  
  theta = wb_inertial_unit_get_roll_pitch_yaw(sensor);
  theta = theta;
  
  
  setpoint = 1;
  
  spPID.update(real_speed, setpoint);
  spPID.compute();
  
  pitch.update(theta, spPID.output);
  pitch.compute();
  
  speed = -pitch.output; 
  
  %wb_console_print(sprintf('Sensor value: %f\n', msg), WB_STDOUT);
  
  
  wb_motor_set_velocity(wheels(1), speed);
  wb_motor_set_velocity(wheels(2), speed);
  wb_motor_set_velocity(wheels(3), speed);
  wb_motor_set_velocity(wheels(4), speed);
  drawnow;
end