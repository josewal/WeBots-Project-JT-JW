classdef Motor < handle
    properties (SetAccess = private)
        tag (1,1) 
        wb_motor
        speed {mustBeNumeric}
        steps {mustBeNumeric}
        enable_bool
    end
    
    methods
        function this = Motor(tag)
            this.tag = tag;
            this.speed = 0;
            this.steps = inf;
            this.enable_bool = true;
            
            this.wb_motor = wb_robot_get_device(convertStringsToChars(this.tag));
            wb_motor_set_position(this.wb_motor, this.steps);
            wb_motor_set_velocity(this.wb_motor, this.speed);
        end
        
        function run(this)
            if this.enable_bool
                wb_motor_set_velocity(this.wb_motor, this.speed);
            end            
        end
        
        function setSpeed(this, speed)
            this.speed = speed;
        end
        
        function enable(this)
        this.enable_bool = true;
        end
        
        function disable(this)
        this.enable_bool = false;
        end
        
    end
end

        