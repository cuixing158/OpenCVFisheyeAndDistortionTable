function [undistortImg,mapX,mapY,camIntrinsic,newOrigin] = undistortFisheyeImgFromTable(...
    oriImg,K,cameraData,options)
% Brief: 直接根据畸变表对图像进行去畸变
% Details:
%    数据表仅需入射角+出射角(或者提供畸变real_height和ref_height可计算出射角)+
%    焦距(或者提供畸变real_height和ref_height可计算焦距)即可，此函数支持C代码生成，快速高效
%    以principle point为中心，畸变逐渐增大。经验值sensorRatio = 0.003mm/pixel.
%
% Syntax:
%     [undistortImg,mapX,mapY] = undistortImgFast(oriImg,K,cameraData)
%
% Inputs:
%    oriImg - [m,n] size,[any] type,Description
%    K - [3,3] size,[double] type,fisheye camera intrinsic matrix,
%       [fx,0,cx;
%       0,fy,cy;
%       0,0,1] format
%    cameraData - [m,n] size,[table] type,必须包含入射角angle和出射角angle_d域名
%    Outputview - [1,1] size,[string] type,Size of the output image, specified
%                 as either 'same','full', or 'valid'.
%    ScaleFactor - [1,2] size, [double] type,Scale factor for the focal
%                length of a virtual camera perspective,in pixels, specified
%                as  [sx sy] vector. Specify a vector to scale the x and y axes
%                individually. Increase the scale to zoom in the perspective
%                of the camera view.
%
% Outputs:
%    undistortImg - [M,N] size,[double] type,无畸变图像
%    mapX - [M,N] size,[double] type,映射到原图oriImg的x的坐标
%    mapY - [M,N] size,[double] type,映射到原图oriImg的y的坐标
%    camIntrinsic - [1,1] size, [cameraIntrinsics] type，等价的完美针孔相机内参
%    newOrigin - [1,2] size, [double] type,新原点坐标,使用undistortFisheyePtsFromTable
%                  函数返回的无畸变点应当减去此值可得到在去畸变图像上的坐标。
%
% Example:
% %% 鱼眼图像去畸变
% oriImage = imread("fisheye.jpg");
% cameraParaPath = "distortionTable.xlsx";
% sensorRatio = 0.003;% 由厂家提供，单位 mm/pixel

% cameraData = readtable(cameraParaPath,VariableNamingRule='preserve');% 第一列为入射角，单位：度，第二列为实际畸变量长度,单位：mm，第三列为理想参考投影长度，单位：mm
% angleIn = cameraData{:,1};% 入射角,单位：度
% focal = mean(cameraData{:,2}./tand(angleIn));% 焦距，单位:mm
% angleOut = atan2d(cameraData{:,3},focal);% 出射角，单位：度，与入射角一一对应
% cameraData = table(angleIn,angleOut,VariableNames = ["angle","angle_d"]);
% [h,w,~] = size(oriImage);

% K = [focal/sensorRatio,0,w/2;
%     0,focal/sensorRatio,h/2;
%     0,0,1];
% scalarSXY = [1,1];% 改变输出图像大小，不影响其他计算
% [undistortImg,mapX,mapY,camIntrinsic]  = undistortFisheyeImgFromTable(oriImage,...
%     K,cameraData,OutputView="same",ScaleFactor=scalarSXY);
% % 后续类似畸变图像直接使用mapX,mapY映射即可，如下语句
% undistortImage = images.internal.interp2d(distortionImage,mapX,mapY,"linear",0, false);
%
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         24-Jun-2022 19:02:23
%
% Implementation In Matlab R2022b
% Copyright © 2022 TheMatrix.All Rights Reserved.
%
arguments
    oriImg
    K (3,3) double
    cameraData table
    options.OutputView (1,:) char {mustBeMember(options.OutputView,...
        {'same','full','valid'})}= 'same'
    options.ScaleFactor (1,2) double {mustBePositive}= [1,1]
end
sx = options.ScaleFactor(1);
sy = options.ScaleFactor(2);

cx = K(1,3);
cy = K(2,3);
flength = [K(1,1),K(2,2)];

m = tand(cameraData.angle);
n = tand(cameraData.angle_d);

% fill out outliers
m(abs(m)>400)=NaN;
n(abs(n)>400)=NaN;
m = fillmissing(m,"nearest");
n = fillmissing(n,"nearest");

refHeight = mean(flength).*m;
realHeight = mean(flength).*n;

[h,w,~] = size(oriImg);
bwImg = ones(h,w,'logical');
B = bwtraceboundary(bwImg,[1,1],'E');% 畸变图像所有边界点，顺时针逐个取点坐标
B(end,:) = [];% last one is dumplicate to first one
edgePts = [B(:,2),B(:,1)];

% 获取矫正图像宽和高
if strcmp(options.OutputView,"same")
    newOrigin = round([-w/2,-h/2]);% 无畸变图像中心点为原点的坐标
    xlim = [newOrigin(1)+1,newOrigin(1)+w]-newOrigin(1);% 无畸变图像以左上角点为原点的坐标
    ylim = [newOrigin(2)+1,newOrigin(2)+h]-newOrigin(2);
elseif strcmp(options.OutputView,"valid")
    distortL = vecnorm(edgePts-[cx,cy],2,2);
    undistortL = interp1(realHeight,refHeight,distortL,"linear",refHeight(end));
    edgesX = (edgePts(:,1)-cx).*undistortL./distortL;
    edgesY = (edgePts(:,2)-cy).*undistortL./distortL;
    undistortPts = [edgesX,edgesY];% 无畸变图像中心点为原点的坐标

    % 求不规则四条曲线边轮廓的最大内接矩形算法
    % 使用4条边的种子边缘向内扩展算法，避免外延扩展算法效率过低，算法复杂度与边缘畸变程度正相关
    topEdgePoints = undistortPts(1:w,:);
    rightEdgePoints = undistortPts(w:w+h-1,:);
    bottomEdgePoints = undistortPts(w+h-1:2*w+h-2,:);
    leftEdgePoints = undistortPts(2*w+h-2:end,:);

    xmin = max(leftEdgePoints(:,1));
    xmax = min(rightEdgePoints(:,1));
    ymin = max(topEdgePoints(:,2));
    ymax = min(bottomEdgePoints(:,2));

    newOrigin = [xmin,ymin];% 无畸变图像中心点为原点的坐标
    xlim = [xmin,xmax]-newOrigin(1); % 无畸变图像以左上角点为原点的坐标
    ylim = [ymin,ymax]-newOrigin(2);
else % "full"
    distortL = vecnorm(edgePts-[cx,cy],2,2);
    undistortL = interp1(realHeight,refHeight,distortL,"linear",refHeight(end));
    edgesX = (edgePts(:,1)-cx).*undistortL./distortL;
    edgesY = (edgePts(:,2)-cy).*undistortL./distortL;
    undistortPts = [edgesX,edgesY];

    newOrigin = [min(undistortPts(:,1)),min(undistortPts(:,2))];
    xlim = [min(undistortPts(:,1)),max(undistortPts(:,1))]-newOrigin(1);
    ylim = [min(undistortPts(:,2)),max(undistortPts(:,2))]-newOrigin(2);
end
principlePt = ([0,0]-newOrigin); %等价的针孔相机主点坐标
U0 = principlePt(1);
V0 = principlePt(2);

% 鱼眼图像插值去畸变,避免矫正图像过大，耗时较多，应当考虑适当缩放
xlim = sx.*xlim;
ylim = sy.*ylim;
[X,Y] = meshgrid(xlim(1):xlim(end),ylim(1):ylim(end)); % X,Y是无畸变缩放图像像素点,当sx,sy小于1，从而减少X,Y数量，提高插值速度
X = X./sx;
Y = Y./sy;
undistortD = sqrt((X-U0).^2+(Y-V0).^2);
distortD = interp1(refHeight,realHeight,undistortD,"linear",refHeight(end));% 以1维插值代替查表操作
mapX = (X-U0).*distortD./(undistortD+eps)+cx;% 避免除数为0
mapY = (Y-V0).*distortD./(undistortD+eps)+cy;% 避免除数为0

undistortImg = images.internal.interp2d(oriImg,mapX,mapY,...
        "linear",255, false);% 与生成的C代码mex文件一样的执行速度

% 等价的pinhole camera intrinsics
focalLen = [sx,sy].*flength;
principlePoint = [sx,sy].*principlePt;
imageSize = size(undistortImg,[1,2]);
camIntrinsic = cameraIntrinsics(focalLen,principlePoint,imageSize);
end