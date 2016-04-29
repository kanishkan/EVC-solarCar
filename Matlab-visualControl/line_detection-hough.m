%% Clear work space
clear all;
close all;
clc;
img = 'test_case\raw.jpg';
%% Configuration
ROI_size = 120;
LaneSize = 2;
firFilter = vision.ImageFilter( ...
                    'Coefficients', [-1 0 1], ...
                    'OutputSize', 'Same as first input', ...
                    'PaddingMethod', 'Replicate', ...
                    'Method', 'Correlation');
LocalMax = vision.LocalMaximaFinder( ...
                        'MaximumNumLocalMaxima', LaneSize, ...
                        'NeighborhoodSize', [301 81], ...
                        'Threshold', 1, ...
                        'HoughMatrixInput', true, ...
                        'IndexDataType', 'uint16');         
houghLines = vision.HoughLines('SineComputation', 'Trigonometric function');
%% Read image

img = imread(img);

% ROI processing
img = img(ROI_size+1:end,:,:);
I = rgb2gray(img); 
filtered_img = step(firFilter,I);

% Convert to black&white
filtered_img(filtered_img<0)=0;
filtered_img(filtered_img>1)=1;
BW = step(vision.Autothresholder, filtered_img);
[H,T,R] = hough(BW);
if(0)
    Lines = step(LocalMax, H);
    Count = size(Lines,1);
    % Select line
    Line = [T(Lines(:, 1)); R(Lines(:, 2))];
    Enable = [ones(1,Count) zeros(1, LaneSize-Count)];

    recon = step(houghLines, Line(2,:), Line(1,:), I);
    lines = houghlines(BW,T,R,Lines);
else
    figure;
    imshow(H,[],'XData',T,'YData',R,...
                'InitialMagnification','fit');
    xlabel('\theta'), ylabel('\rho');
    axis on, axis normal, hold on;
    title('Hough Transform');
    %detect four peaks
    P  = houghpeaks(H,LaneSize,'NHoodSize',[101 51]);   %  NHoodSize -> Important
    hold on; 
    plot( T( P(:,2) ), R( P(:,1) ), 's', 'color', 'green'); 
    lines = houghlines(BW,T,R,P);
end

figure;
subplot(2,3,1);imshow(img); title('Original (only ROI)');
subplot(2,3,2);imshow(I);title('Gray scale');
subplot(2,3,3);imshow(filtered_img);title('Filtered (Before threshold)');
subplot(2,3,4);imshow(BW);title('Filtered (After threshold)');
subplot(2,3,5);imshow(H);title('Hough');
subplot(2,3,6);title('Line detection');
imshow(img), hold on
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
