%
%=====================================================================================
%       Filename:  SuperSightNLOSAlgorithm.m
% 
%    Description:  Input the setup and measured values to find the NLOS 6DoF of triangular tag array.
%        Version:  1.0
%
%         Author:  Kang Min Bae, Hankyeol Moon
%         Email :  smilelabkaist@gmail.com
%   Organization:  Smart and Mobile Systems (Smile) Lab @ KAIST 
%                  https://smile.kaist.ac.kr/
%
%   Copyright (c)  Smart and Mobile Systems (Smile) Lab @ KAIST
%   This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
%=====================================================================================
%

digits(10)

%% Input Values Measured From Radar

% =====================================================================================
%   - All angles in radian, range in meter.
%   - Positive x direction is the radar face.
%   - Azimuth is angle from the x axis to projection on the x,y plane, Elevation is angle to the x,y plane.
% =====================================================================================


AoAAZp1 = 0.5880; % AoA Azimuth for single-reflector path 1.
AoAAZp2 = -0.5880;
AoAAZp3 = 0;

AoAELp1 = 0; % AoA Elevation for single-reflector path 1.
AoAELp2 = 0;
AoAELp3 = -0.3948;

ToFp1t1 = 3.6056; % ToF to tag 1 for single-reflector path 1.
ToFp1t2 = 3.6620;
ToFp1t3 = 3.6069;

ToFp2t1 = 3.6056;
ToFp2t2 = 3.5511;
ToFp2t3 = 3.6069;

ToFp3t1 = 3.2500;
ToFp3t2 = 3.2515;
ToFp3t3 = 3.2898;

%% Input Triangular Tag Array Geometry

% =====================================================================================
%   - The code assumes triangular tag array with a right triangle, with tag2-tag1-tag3 forming a right angle.
% =====================================================================================

TagDist12 = 0.1; % tag1-tag2 distance
TagDist31 = 0.1; % tag1-tag3 distance

%% Input Postion Of Radars

% =====================================================================================
%   - Radar1 is at [0, 0, 0]. For single-radar setup, Radar2 and Radar3 are also at [0, 0, 0].
% =====================================================================================

Radar2Pos = [0, 0, 0];
Radar3Pos = [0, 0, 0];

%% Find virtual radar position using tag array geometry

%for path 1%
syms tempx tempy tempz

tempeqn1 = (tempx-TagDist12)^2 + tempy^2 + tempz^2 == ToFp1t2^2;
tempeqn2 = tempx^2 + (tempy-TagDist31)^2 + tempz^2 == ToFp1t3^2;
tempeqn3 = tempx^2 + tempy^2 + tempz^2 == ToFp1t1^2;

tempsol = vpasolve([tempeqn1, tempeqn2, tempeqn3], [tempx, tempy, tempz]);

Xp1 = double(tempsol.tempx(1));
Yp1 = double(tempsol.tempy(1));
Zp1 = double(abs(tempsol.tempz(1)));

%for path 2%
syms tempx tempy tempz

tempeqn1 = (tempx-TagDist12)^2 + tempy^2 + tempz^2 == ToFp2t2^2;
tempeqn2 = tempx^2 + (tempy-TagDist31)^2 + tempz^2 == ToFp2t3^2;
tempeqn3 = tempx^2 + tempy^2 + tempz^2 == ToFp2t1^2;

tempsol = vpasolve([tempeqn1, tempeqn2, tempeqn3], [tempx, tempy, tempz]);


Xp2 = double(tempsol.tempx(1));
Yp2 = double(tempsol.tempy(1));
Zp2 = double(abs(tempsol.tempz(1)));

%for path 3%
syms tempx tempy tempz

tempeqn1 = (tempx-TagDist12)^2 + tempy^2 + tempz^2 == ToFp3t2^2;
tempeqn2 = tempx^2 + (tempy-TagDist31)^2 + tempz^2 == ToFp3t3^2;
tempeqn3 = tempx^2 + tempy^2 + tempz^2 == ToFp3t1^2;

tempsol = vpasolve([tempeqn1, tempeqn2, tempeqn3], [tempx, tempy, tempz]);

Xp3 = double(tempsol.tempx(1));
Yp3 = double(tempsol.tempy(1));
Zp3 = double(abs(tempsol.tempz(1)));

%% Solve Equation

syms Pitch Yaw Roll TagX TagY TagZ
minRange = min([ToFp1t1,ToFp2t1,ToFp3t1]);

eqn1 = TagX == (TagZ + ToFp1t1 * sin(asin((-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)/sqrt((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)^2 + (sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)^2 + (-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)^2))))/(sin(AoAELp1) + sin(asin((-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)/sqrt((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)^2 + (sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)^2 + (-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)^2))))*(cos(AoAELp1)*cos(AoAAZp1) + cos(asin((-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)/sqrt((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)^2 + (sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)^2 + (-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)^2)))*cos(angle((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)+1j*(sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)))) - ToFp1t1*(cos(asin((-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)/sqrt((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)^2 + (sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)^2 + (-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)^2)))*cos(angle((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)+1j*(sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1))));
eqn2 = TagY == (TagZ + ToFp1t1 * sin(asin((-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)/sqrt((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)^2 + (sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)^2 + (-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)^2))))/(sin(AoAELp1) + sin(asin((-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)/sqrt((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)^2 + (sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)^2 + (-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)^2))))*(cos(AoAELp1)*sin(AoAAZp1) + cos(asin((-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)/sqrt((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)^2 + (sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)^2 + (-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)^2)))*sin(angle((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)+1j*(sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)))) - ToFp1t1*(cos(asin((-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)/sqrt((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)^2 + (sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1)^2 + (-sin(Pitch)*Xp1 + cos(Pitch)*sin(Roll)*Yp1 + cos(Pitch)*cos(Roll)*Zp1)^2)))*sin(angle((cos(Yaw)*cos(Pitch)*Xp1 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp1 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp1)+1j*(sin(Yaw)*cos(Pitch)*Xp1 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp1 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp1))));
eqn3 = (TagX+Radar2Pos(1)) == ((TagZ+Radar2Pos(3)) + ToFp2t1 * sin(asin((-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)/sqrt((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)^2 + (sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)^2 + (-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)^2))))/(sin(AoAELp2) + sin(asin((-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)/sqrt((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)^2 + (sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)^2 + (-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)^2))))*(cos(AoAELp2)*cos(AoAAZp2) + cos(asin((-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)/sqrt((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)^2 + (sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)^2 + (-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)^2)))*cos(angle((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)+1j*(sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)))) - ToFp2t1*(cos(asin((-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)/sqrt((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)^2 + (sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)^2 + (-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)^2)))*cos(angle((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)+1j*(sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2))));
eqn4 = (TagY+Radar2Pos(2)) == ((TagZ+Radar2Pos(3)) + ToFp2t1 * sin(asin((-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)/sqrt((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)^2 + (sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)^2 + (-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)^2))))/(sin(AoAELp2) + sin(asin((-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)/sqrt((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)^2 + (sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)^2 + (-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)^2))))*(cos(AoAELp2)*sin(AoAAZp2) + cos(asin((-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)/sqrt((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)^2 + (sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)^2 + (-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)^2)))*sin(angle((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)+1j*(sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)))) - ToFp2t1*(cos(asin((-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)/sqrt((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)^2 + (sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2)^2 + (-sin(Pitch)*Xp2 + cos(Pitch)*sin(Roll)*Yp2 + cos(Pitch)*cos(Roll)*Zp2)^2)))*sin(angle((cos(Yaw)*cos(Pitch)*Xp2 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp2 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp2)+1j*(sin(Yaw)*cos(Pitch)*Xp2 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp2 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp2))));
eqn5 = (TagX+Radar3Pos(1)) == ((TagZ+Radar3Pos(3)) + ToFp3t1 * sin(asin((-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)/sqrt((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)^2 + (sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)^2 + (-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)^2))))/(sin(AoAELp3) + sin(asin((-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)/sqrt((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)^2 + (sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)^2 + (-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)^2))))*(cos(AoAELp3)*cos(AoAAZp3) + cos(asin((-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)/sqrt((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)^2 + (sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)^2 + (-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)^2)))*cos(angle((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)+1j*(sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)))) - ToFp3t1*(cos(asin((-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)/sqrt((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)^2 + (sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)^2 + (-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)^2)))*cos(angle((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)+1j*(sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3))));
eqn6 = (TagY+Radar3Pos(2)) == ((TagZ+Radar3Pos(3)) + ToFp3t1 * sin(asin((-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)/sqrt((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)^2 + (sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)^2 + (-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)^2))))/(sin(AoAELp3) + sin(asin((-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)/sqrt((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)^2 + (sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)^2 + (-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)^2))))*(cos(AoAELp3)*sin(AoAAZp3) + cos(asin((-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)/sqrt((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)^2 + (sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)^2 + (-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)^2)))*sin(angle((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)+1j*(sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)))) - ToFp3t1*(cos(asin((-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)/sqrt((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)^2 + (sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3)^2 + (-sin(Pitch)*Xp3 + cos(Pitch)*sin(Roll)*Yp3 + cos(Pitch)*cos(Roll)*Zp3)^2)))*sin(angle((cos(Yaw)*cos(Pitch)*Xp3 + (cos(Yaw)*sin(Pitch)*sin(Roll)-sin(Yaw)*cos(Roll))*Yp3 + (cos(Yaw)*sin(Pitch)*cos(Roll)+sin(Yaw)*sin(Roll))*Zp3)+1j*(sin(Yaw)*cos(Pitch)*Xp3 + (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Yaw)*cos(Roll))*Yp3 + (sin(Yaw)*sin(Pitch)*cos(Roll)-cos(Yaw)*sin(Roll))*Zp3))));

sol = vpasolve([eqn1, eqn2, eqn3, eqn4, eqn5, eqn6], [Yaw, Pitch, Roll, TagX, TagY, TagZ], [-pi,pi; -pi/2,pi/2; -pi,pi; 0,minRange; -minRange,minRange; -minRange,minRange]);


localizationResult = double([sol.TagX sol.TagY sol.TagZ rad2deg(sol.Yaw) rad2deg(sol.Pitch) rad2deg(sol.Roll)])





