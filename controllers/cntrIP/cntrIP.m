%desktop;


%======================================================%
%INTIALIZATION OF SENSORS AND MOTORS
TIME_STEP = 32;
t = 0;

wb_keyboard_enable(TIME_STEP);

IMU = wb_robot_get_device(convertStringsToChars("IMU"));
wb_inertial_unit_enable(IMU, TIME_STEP);

acc = wb_robot_get_device(convertStringsToChars("accelerometer"));
wb_accelerometer_enable(acc, TIME_STEP);


camera = wb_robot_get_device(convertStringsToChars("camera"));
wb_camera_enable(camera, TIME_STEP);

motor_tags = ["arm motor", "wheel", "yaw motor"];
for i = 1:length(motor_tags);
    motors(i) = Motor(motor_tags(i));
end
%======================================================%


%======================================================%
%INTIALIZATION OF PID CONTROLERS

%FORWARD VELOCITY PID CONTROLLER
velocityPID = PID(0.02, 0.0001, 0.01);
velocityPID.enable()
velocityPID.setLimits(-1,1)
velocityPID.setSetpoint(0)
desired_velocity = 0;

%PITCH PID CONTROLLER
pitchPID = PID(50, 50, 10);
pitchPID.enable()
pitchPID.setLimits(-50,50)

%ROLL PID CONTROLLER
rollPID = PID(700, 100, 700);
rollPID.enable()
rollPID.setLimits(-100,100)

%YAW PID CONTROLLER
yawPID = PID(5, 0.25, 1);
yawPID.enable()
yawPID.setLimits(-50,50)
desired_yaw = -pi/2

%BANKING INTO TURNS PID CONTROLLER
bankPID = PID(0.12, 0.15, 0.02);
bankPID.disable()
bankPID.setLimits(-0.2,0.2)

%BANK ANGLE PID CONTROLLER
bankAnglePID = PID(0.0005, 0.001, 0.0001);
bankAnglePID.enable()
bankAnglePID.setLimits(-0.2,0.2)

%PATH OFFSET PID CONTROLLER
pathPID = PID(0.001, 0.1, 0.01);
pathPID.enable()
pathPID.setLimits(-0.05,0.05)
%======================================================%


%INITIALIZATION FOR LOWPASS FILTER
balance_angle_LF = [0,0];

%INITIALIZATION FOR YAW OVERFLOW
rotations = 0;
prev_pitch_roll_yaw = [0,0,-pi/2];

hold on



%======================================================%
%======================================================%
%MAIN LOOP

while wb_robot_step(TIME_STEP) ~= -1
    t = [t(end), t(end) + 1];
    
    if t(end) == 20
        bankPID.enable();
    end
    
    key = [wb_keyboard_get_key(), wb_keyboard_get_key()];
    for i = 1:2
        switch key(i)
            case 315
                desired_velocity = desired_velocity + 0.25;
            case 317
                desired_velocity = desired_velocity - 0.25;
            case 314
                desired_yaw = desired_yaw + 0.03;
            case 316
                desired_yaw = desired_yaw - 0.03;
        end
    end
    
    
    input_image = wb_camera_get_image(camera);
    [path_offset, path_angle, endpoint_offset, gotLine] = lineRecognition(input_image);
    
    pitch_roll_yaw = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    d = prev_pitch_roll_yaw(3) - pitch_roll_yaw(3);
    
    if d > pi
        rotations = rotations + 2*pi;
    elseif d < -pi
        rotations = rotations - 2*pi;
    end

    prev_pitch_roll_yaw = pitch_roll_yaw;
    pitch_roll_yaw(3) = pitch_roll_yaw(3) + rotations;
    
    
    acc_x_y_z = wb_accelerometer_get_values(acc);
    balance_angle = getAccAngle(acc_x_y_z);
    
    balance_angle_LF = [balance_angle_LF(end),...
        0.9*balance_angle_LF(end) + 0.1*balance_angle];
    
    
    %PID CONTROLLERS COMPUTATION
    pathPID.setSetpoint(path_offset);
    pathPID.update(0);
    if path_offset * path_angle > 0
    desired_yaw = desired_yaw + 4*pathPID.output;
    else
    desired_yaw = desired_yaw + pathPID.output;
    end
    
    desired_velocity = 0.01 * (32 - abs(endpoint_offset));
    
    bankAnglePID.setSetpoint(0);
    bankAnglePID.update(motors(1).speed);
    desired_bank = -bankAnglePID.output;
    
    bankPID.setSetpoint(desired_bank);
    bankPID.update(-balance_angle_LF(end));
    desired_roll = bankPID.output;
    
    velocity = motors(2).speed * 0.05;
    velocityPID.setSetpoint(desired_velocity);
    velocityPID.update(velocity(end));
    desired_pitch = -velocityPID.output;
    
    pitchPID.setSetpoint(desired_pitch);
    pitchPID.update(pitch_roll_yaw(1));
    motors(2).setSpeed(pitchPID.output);
    
    rollPID.setSetpoint(desired_roll);
    rollPID.update(pitch_roll_yaw(2));
    motors(1).setSpeed(-rollPID.output);
    
    yawPID.setSetpoint(desired_yaw);
    yawPID.update(pitch_roll_yaw(3));
    motors(3).setSpeed(yawPID.output);
    
    
    for i = 1:length(motors)
        motors(i).run();
    end
    
    drawnow;
end

%======================================================%
%======================================================%