TIME_STEP = 64;

wheels = [];
wheels_names = [ "wheel1", "wheel2", "wheel3", "wheel4" ];
for i = 1:4
  wheels(i) = wb_robot_get_device(convertStringsToChars(wheels_names(i)));
  wb_motor_set_position(wheels(i), inf);
  wb_motor_set_velocity(wheels(i), 0.0);
end

while wb_robot_step(TIME_STEP) ~= -1
  left_speed = 5.0;
  right_speed = 2.5;
  
  wb_motor_set_velocity(wheels(1), left_speed);
  wb_motor_set_velocity(wheels(2), right_speed);
  wb_motor_set_velocity(wheels(3), left_speed);
  wb_motor_set_velocity(wheels(4), right_speed);
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end