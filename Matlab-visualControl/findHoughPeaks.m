function [ lines ] = findHoughPeaks( image, numOfPeaks, enablePlot )
%findHoughPeaks - Finds specified number of peak lines in the given image
%   Usage: [ lines ] = findHoughPeaks( image, numOfPeaks, enablePlot )
%   Input: Image array (RGB/Gray scale) & Required number of lines.
%   Return Value: Start and end points of line in following format.
%       1x2 struct array with fields:
%           point1
%           point2
%           theta
%           rho
%   Optional: 'enablePlot' to enable image plot.

%% Config
NeighHoodSize = [101 51];
firFilter = vision.ImageFilter( ...
                    'Coefficients', [-1 0 1], ...
                    'OutputSize', 'Same as first input', ...
                    'PaddingMethod', 'Replicate', ...
                    'Method', 'Correlation');

%% Line detection
if(size(image,3) ~=1)
    I = rgb2gray(image); 
else
    I=image;
end
filtered_img = step(firFilter,I);

% Convert to black&white
filtered_img(filtered_img<0)=0;
filtered_img(filtered_img>1)=1;
BW = step(vision.Autothresholder, filtered_img);
[H,T,R] = hough(BW);

% detect peaks
P  = houghpeaks(H,numOfPeaks,'NHoodSize',NeighHoodSize); % Threshold also matters
lines = houghlines(BW,T,R,P);

%% Plot the detected lines
if(enablePlot)
    % hough Transform
    figure;
    imshow(H,[],'XData',T,'YData',R,...
                'InitialMagnification','fit');
    xlabel('\theta'), ylabel('\rho');
    axis on, axis normal, hold on;
    title('Hough Transform');
    
    % Lines
    figure;
    imshow(image), hold on
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

       % Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

       % Determine the endpoints of the longest line segment
       len = norm(lines(k).point1 - lines(k).point2);
       if ( len > max_len)
          max_len = len;
          xy_long = xy;
       end
    end
end
end

