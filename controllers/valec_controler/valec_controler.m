% MATLAB controller for Webots
% File:          valec_controler.m
% Date:
% Description:
% Author:
% Modifications:


%desktop;
%keyboard;


TIME_STEP = 32;

emitter = wb_robot_get_device(convertStringsToChars("emitter"));
wb_emitter_set_channel(emitter, 1);

gyro = wb_robot_get_device(convertStringsToChars("gyro"));
wb_gyro_enable(gyro, TIME_STEP)

while wb_robot_step(TIME_STEP) ~= -1

  
  gyro_data = wb_gyro_get_values(gyro);
  x = gyro_data(2);
  speed = (0.4*pi)*(x/(2*pi));
  wb_emitter_send(emitter, speed);
  drawnow;

end
