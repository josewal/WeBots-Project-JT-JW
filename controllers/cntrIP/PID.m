classdef PID < handle
    
    properties (SetAccess = private)
        p = zeros(1, 50);
        i = zeros(1, 50);
        d = zeros(1, 50);
        output {mustBeNumeric};
        error {mustBeNumeric};
    end
    
    properties
        Kp {mustBeNumeric};
        Ki {mustBeNumeric};
        Kd {mustBeNumeric};
        input {mustBeNumeric};
        setpoint {mustBeNumeric};
    end
    
    
    methods
        function this = PID(Kp,Ki,Kd)
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.setpoint = 0;
            this.input = 0;
        end
        
        function compute(this)
            this.p = [this.p(2:end), this.error];
            this.i = [this.i(2:end), sum(this.p(end-30:end))];
            this.d = [this.d(2:end), this.error - this.p(end-1)];
            
            this.output = (this.Kp * this.p(end) / 1)...
                + (this.Ki * this.i(end) / 1)...
                + (this.Kd * this.d(end) / 1);
        end
        
        function update(this,input,setpoint)
            this.setpoint = setpoint;
            this.input = input;
            this.error = this.input - this.setpoint;
        end    
    end
end