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

p = zeros(1, 50);
i = zeros(1, 50);
d = zeros(1, 50);

Kp = 50;
Ki = 10;
Kd = 0;

input = [];
setpoint = 0.1;

p = zeros(1, 50);
i = zeros(1, 50);
d = zeros(1, 50);

Kp = 50;
Ki = 10;
Kd = 0;

input = [];
setpoint = 0.1;



msg = [];
while wb_robot_step(TIME_STEP) ~= -1

  while wb_receiver_get_queue_length(receiver) > 0
  msg = wb_receiver_get_data(receiver, 'double');
  wb_receiver_next_packet(receiver);
  end

  real_speed = msg;
  theta = wb_inertial_unit_get_roll_pitch_yaw(sensor);
  theta = theta(1);
  
  input = theta - setpoint;
  
  p = [p(2:end), input];
  i = [i(2:end), sum(p(end-49:end))];
  d = [d(2:end), input - p(end-1)];
  
  speed = (Kp * p(end) / 1) + (Ki * i(end) / 1) + (Kd * d(end) / 1);
  speed = -speed; 
  
  wb_console_print(sprintf('Sensor value: %f\n', msg), WB_STDOUT);
  
  %data = [data(2:end), input];
  
  wb_motor_set_velocity(wheels(1), speed);
  wb_motor_set_velocity(wheels(2), speed);
  wb_motor_set_velocity(wheels(3), speed);
  wb_motor_set_velocity(wheels(4), speed);
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end