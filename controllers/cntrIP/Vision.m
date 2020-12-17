classdef Vision < handle
    properties
        path_angle
        start_offset
        mid_offset
        end_offset
        lines
        raw_image
        BW_image
    end
    
    methods
        
        function this = Vision()
        end
        
        function getImage(this, input_image)
            this.raw_image = input_image;
            [BW, ~] = path4Mask(input_image);
            this.BW_image = imfill(BW, 'holes');
        end
        
        function findLines(this)
            [H,T,R] = hough(BW);
            P  = houghpeaks(H,20,'threshold',ceil(0.3*max(H(:))));
            
            this.lines = houghlines(BW,T,R,P,'FillGap',10,'MinLength',30);
        end
        
        function findMainLine(this)
            
            endpoint_sum = 0;
            startpoint_sum = 0;
            for i = 1:length(this.lines)
                endpoint_sum = endpoint_sum + this.lines(i).point1;
                startpoint_sum = startpoint_sum + this.lines(i).point2;
            end
            
            n = length(this.lines);
            
            endpoint = endpoint_sum/n;
            startpoint = startpoint_sum/n;
            midpoint = (startpoint + endpoint)/2;
            x = [startpoint(1), midpoint(1), endpoint(1)];
            y = [startpoint(2),midpoint(2), endpoint(2)];
            
            
            this.start_offset = x(1) - 32;
            this.mid_offset = x(2) - 32;
            this.end_offset = x(3) - 32;
            
            u = [0,-32];
            v = [diff([x(1), x(2)]), diff([y(1), y(2)])];
            
            angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
            
            if endpoint(1) < 32
                angle = - angle;
            end
        end
    end
end
