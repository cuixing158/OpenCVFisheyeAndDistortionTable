function [mapX,mapY,undistortPts, distortPts] = initUndistortRectifyMapOpenCV(K, opencvCoeffs,newCameraMatrixK,newImageSize)
% Brief: 由opencv鱼眼畸变系数得到映射和坐标点对应，功能等同于opencv的initUndistortRectifyMap函数
% Details:
%    None
% 
% Syntax:  
%     [mapX,mapY,undistortPts, distortPts] = initUndistortRectifyMapOpenCV(w, h, K, opencvCoeffs)
% 
% Inputs:
%    K - [3,3] size,[double] type,fisheye camera intrinsic,
%       [fx,0,cx;
%       0,fy,cy;
%       0,0,1] format
%    opencvCoeffs - [1,4] size,[double] type,opencv fisheye coeffs
%    newCameraMatrixK - [3,3] size,[double] type,new fisheye camera intrinsic,
%       [fx,0,cx;
%       0,fy,cy;
%       0,0,1] format
%    newImageSize - [1,2],[double] type,new image [height,width]
% 
% Outputs:
%    mapX - [h,w] size,[double] type,Description
%    mapY - [h,w] size,[double] type,Description
%    undistortPts - [h*w,2] size,[double] type,Description
%    distortPts - [h*w,2] size,[double] type,Description
% 
% 
% See also: None

% Author:                          cuixingxing
% Email:                           cuixingxing150@gmail.com
% Created:                         27-Sep-2022 07:42:05
% Implementation In Matlab R2022a
% Copyright © 2022 TheMatrix.All Rights Reserved.
%
arguments 
    K (3,3) {mustBeNumeric}
    opencvCoeffs (1,4) {mustBeNumeric}
    newCameraMatrixK (3,3) {mustBeNumeric}
    newImageSize (1,2) {mustBeNumeric}
end

% coeff convert to matlab
centerY = newImageSize(1)/2; % ensure distortion lie in center of image 
centerX = newImageSize(2)/2; % ensure distortion lie in center of image
offsetX = K(1,3)-centerX;
offsetY = K(2,3)-centerY;
[undistortX,undistortY] = meshgrid(1+offsetX:newImageSize(2)+offsetX,1+offsetY:newImageSize(1)+offsetY);
undistortPts = [undistortX(:),undistortY(:)];

undistortPtsHomo = [undistortPts';
    ones(1,prod(newImageSize))]; % 3*cols size
undistortCameraPts = newCameraMatrixK\undistortPtsHomo; % 3*cols size
undistortCameraPts = undistortCameraPts./undistortCameraPts(end,:);% 3*cols size

r = vecnorm(undistortCameraPts(1:2,:),2,1); % 1*cols size
theta = atan(r);

r_d = theta.*(1+opencvCoeffs(1)*theta.^2+opencvCoeffs(2)*theta.^4+...
    opencvCoeffs(3)*theta.^6+opencvCoeffs(4)*theta.^8); % r_d非theta_d

r(r<=10^(-8))=1;
scale =r_d./r;
u = K(1,1)*undistortCameraPts(1,:).*scale+ K(1,3);
v = K(2,2)*undistortCameraPts(2,:).*scale + K(2,3);
distortPts = [u',v'];% rows*2

mapX = reshape(distortPts(:,1),newImageSize(1),newImageSize(2));
mapY = reshape(distortPts(:,2),newImageSize(1),newImageSize(2));
end
