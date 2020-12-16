function [offset, angle, gotLine] = lineRecognition(input_image)
    [BW, ~] = path4Mask(input_image);
    
    BW = imfill(BW, 'holes');
    %se = strel('disk', 9);
    %BW = imerode(BW, se);
    
    [H,T,R] = hough(BW);
    P  = houghpeaks(H,20,'threshold',ceil(0.3*max(H(:))));
    
    lines = houghlines(BW,T,R,P,'FillGap',10,'MinLength',30);
    imshow(BW, 'InitialMagnification', 'fit'), hold on
    
    if length(lines) < 5
    gotLine = false;
    else
    gotLine = true;
    end
    
    end_x_sum = 0;
    for i = 1:length(lines)
        end_x_sum = end_x_sum + lines(i).point1(1);
    end
    
    start_x_sum = 0;
    for i = 1:length(lines)
        start_x_sum = start_x_sum + lines(i).point2(1);
    end
    
    x_end_midd = end_x_sum/length(lines);
    x_start_midd = start_x_sum/length(lines);
    x = [x_start_midd,x_end_midd];
    y = [64, 32];
    
    for k = 1:length(lines)
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','green');
    end
    
    plot(x, y,'-','LineWidth',2,'Color','red');
    plot([32,32], y,'-','LineWidth',2,'Color','blue');
    hold off
    
    offset = x(1) - 32;
    
    u = [0,-32];
    v = [diff(x), diff(y)];
    
    angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
end