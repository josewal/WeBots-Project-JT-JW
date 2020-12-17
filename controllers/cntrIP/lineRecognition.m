function [offset, mid_offset, end_offset, angle, gotLine] = lineRecognition(input_image)
[BW, ~] = path4Mask(input_image);
BW = imfill(BW, 'holes');


[H,T,R] = hough(BW);
P  = houghpeaks(H,20,'threshold',ceil(0.3*max(H(:))));

lines = houghlines(BW,T,R,P,'FillGap',10,'MinLength',30);
%imshow(BW, 'InitialMagnification', 'fit')
%hold on


if length(lines) > 2
    gotLine = true;
    
    
    endpoint_sum = 0;
    startpoint_sum = 0;
    for i = 1:length(lines)
        endpoint_sum = endpoint_sum + lines(i).point1;
        startpoint_sum = startpoint_sum + lines(i).point2;
    end
    
    endpoint = endpoint_sum/length(lines);
    startpoint = startpoint_sum/length(lines);
    midpoint = (startpoint + endpoint)/2;
    x = [startpoint(1), midpoint(1), endpoint(1)];
    y = [startpoint(2),midpoint(2), endpoint(2)];
    
    %for k = 1:length(lines)
    %     xy = [lines(k).point1; lines(k).point2];
    %    plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','cyan');
   % end
    
    
    %plot(x, y,'-','LineWidth',2,'Color','red');
    %plot([endpoint(1),32], [endpoint(2),endpoint(2)],'-','LineWidth',2,'Color','yellow');
    %plot([midpoint(1),32], [midpoint(2),midpoint(2)],'-','LineWidth',2,'Color','magenta');
    %plot([startpoint(1),32], [startpoint(2),startpoint(2)],'-','LineWidth',2,'Color','blue');
    %plot([32,32], [64,0],'-','LineWidth',2,'Color','green');
    %hold off
    
    offset = x(1) - 32;
    mid_offset = x(2) - 32; 
    end_offset = x(3) - 32;
    u = [0,-32];
    v = [diff([x(1), x(2)]), diff([y(1), y(2)])];
    
    angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
    
    if endpoint(1) < 32
        angle = - angle;
    end
else
    gotLine = false;
    offset = 0;
    angle = 0;
    end_offset = 0;
    mid_offset = 0;
end

end