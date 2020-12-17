function [offset, angle, gotLine, end_offset] = lineRecognition(input_image)
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
    for i = 1:length(lines)
        endpoint_sum = endpoint_sum + lines(i).point1;
    end
    
    
    startpoint_sum = 0;
    for i = 1:length(lines)
        startpoint_sum = startpoint_sum + lines(i).point2;
    end
    
    
    endpoint = endpoint_sum/length(lines);
    startpoint = startpoint_sum/length(lines);
    x = [startpoint(1),endpoint(1)];
    y = [startpoint(2),endpoint(2)];
    
    %for k = 1:length(lines)
   %     xy = [lines(k).point1; lines(k).point2];
    %    plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','green');
    %end
    
    
    %plot(x, y,'-','LineWidth',2,'Color','red');
    %plot([endpoint(1),32], [endpoint(2),endpoint(2)],'-','LineWidth',2,'Color','yellow');
    %plot([startpoint(1),32], [startpoint(2),startpoint(2)],'-','LineWidth',2,'Color','blue');
    %plot([32,32], [64,0],'-','LineWidth',2,'Color','cyan');
    %hold off
    
    offset = x(1) - 32;
    end_offset = x(2) - 32;
    u = [0,-32];
    v = [diff(x), diff(y)];
    
    angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
    
    if endpoint(1) < 32
        angle = - angle;
    end
else
    gotLine = false;
    offset = 0;
    angle = 0;
    end_offset = 0;
end

end