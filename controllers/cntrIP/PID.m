classdef PID < handle
    
    properties (SetAccess = private)
        p = 0;
        i = 0;
        d = [0, 0];
        input {mustBeNumeric};
        setpoint {mustBeNumeric}; 
        output {mustBeNumeric};
        error {mustBeNumeric};
        limits (1,2){mustBeNumeric};
        enable_bool
        Kp {mustBeNumeric};
        Ki {mustBeNumeric};
        Kd {mustBeNumeric};
    end
      
    methods
        function this = PID(Kp,Ki,Kd)
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.setpoint = 0;
            this.input = 0;
            this.limits = [-inf, inf];
            this.enable_bool = true;
        end
        
        function compute(this)
            this.p = this.error;
            this.i = this.i + this.error;
            this.d = [this.d(end), this.error - this.d(end)];
            
            this.output = (this.Kp * this.p)...
                        + (this.Ki * this.i)...
                        + (this.Kd * this.d(end));
            
            if this.output < this.limits(1) % ???
                this.output = this.limits(1);
            elseif this.output > this.limits(2) % ??
                this.output = this.limits(2); 
            end
        end
        
        function update(this,input,setpoint)
            this.setpoint = setpoint;
            this.input = input;
            this.error = this.input - this.setpoint;
            
            if (this.enable_bool)
              this.compute();
            else
              this.output = 0;
            end
        end
        
        function setSetpoint(this,setpoint)
            this.setpoint = setpoint;
        end
        
        function setTunings(this,tunings)
            this.Kp = tunings(1);
            this.Ki = tunings(2);
            this.Kd = tunings(3);
         end
        
        function setLimits(this,min,max)
            this.limits = [min, max]; %??
        end
        
        function enable(this) 
        this.enable_bool = true; 
        end
        
        function disable(this)
        this.enable_bool = false;
        end
    end
end