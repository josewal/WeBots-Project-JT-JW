TIME_STEP = 64;

sensor = wb_robot_get_device(convertStringsToChars("angle_sensor"));
wb_position_sensor_enable(sensor, TIME_STEP);

wheels = [];
wheels_names = [ "wheel1", "wheel2", "wheel3", "wheel4" ];
for i = 1:4
  wheels(i) = wb_robot_get_device(convertStringsToChars(wheels_names(i)));
  wb_motor_set_position(wheels(i), inf);
  wb_motor_set_velocity(wheels(i), 0.0);
end

theta = [];

p = 0; 
i = 0;
d = 0;

Kp = 100;
Ki = 25;
Kd = 0;

input = [];
prev_input = 0;
setpoint = 0;
dir_flag = true

while wb_robot_step(TIME_STEP) ~= -1
  left_speed = 0;
  right_speed = 0;
  
  theta = wb_position_sensor_get_value(sensor);
  input = theta - setpoint;
  
  p = input;
  i = i + input;
  d = input - prev_input;
  
  speed = (Kp * p / 1) + (Ki * i / 1) + (Kd * d / 1);
  
  
  wb_console_print(sprintf('Sensor value: %f\n', theta), WB_STDOUT);
  
  prev_input = input;
  
  if (setpoint > 0.02)
    dir_flag = false;
  elseif (setpoint < -0.02)
    dir_flag = true;
  end
  
  if dir_flag
    setpoint = setpoint + 0.002;
  else
    setpoint = setpoint - 0.002;
  end
  
  wb_motor_set_velocity(wheels(1), speed);
  wb_motor_set_velocity(wheels(2), speed);
  wb_motor_set_velocity(wheels(3), speed);
  wb_motor_set_velocity(wheels(4), speed);
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end