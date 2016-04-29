%% Path Extraction

%% Load workspace
clear all;
close all;
clc;
img = 'test_case\raw.jpg';
img = imread(img);
threshold_distance = 5;     % Have to find using train and error

%% Near field computation
ROI_size = 120;
LaneSize = 2;
%img_near = img(ROI_size+1:end,:,:);
img_near = img;
% detect lines
lines = findHoughPeaks(img_near,LaneSize,true);


%% Intersection

line1=[lines(1).point1;lines(1).point2];
line2=[lines(2).point1;lines(2).point2];
% Invert Y-axis to convert to cartesien plane
line1(:,2) = size(img_near,2)-line1(:,2); 
line2(:,2)= size(img_near,2)-line2(:,2);

% Solve slope equation
figure;
hold all
plot(line1(:,1),line1(:,2));
plot(line2(:,1),line2(:,2));
slope = @(line) (line(2,2) - line(1,2))/(line(2,1) - line(1,1));
m1 = slope(line1);
m2 = slope(line2);
intercept = @(line,m) line(1,2) - m*line(1,1);
b1 = intercept(line1,m1);
b2 = intercept(line2,m2);
xintersect = (b2-b1)/(m1-m2);
yintersect = m1*xintersect + b1;
plot(xintersect,yintersect,'m*','markersize',8);
intersect_point = [xintersect yintersect];

% Find lane
if(line1(:,1)>line2(:,1))
    leftLane = line1;
    rightLane = line2;
else
    leftLane = line2(2,:);
    rightLane = line1(2,:);
end
% Orgin points
leftLane_orgin = leftLane(2,:);
rightLane_orgin = rightLane(2,:);
%% Warning System

% Find distance to orgin point
leftLane_dist = pdist([leftLane_orgin;intersect_point],'euclidean');
rightLane_dist = pdist([rightLane_orgin;intersect_point],'euclidean');

% Deviation
deviation = leftLane_dist-rightLane_dist;

if(deviation > threshold_distance)
    % Moving towards left side
    disp('Inclined towards Left');
elseif(deviation < -threshold_distance)
    % Moving towards right side
    disp('Inclined towards Right');
else
    % on right path
    disp('On the right path');
end