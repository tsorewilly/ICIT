clear; clc; 
start_timer=tic;
PL1=10; RL1=6.4;
L=[5.4 5.4 5.4 5.4 5.4 5.4 5.4 5.4];
ALL=[PL1 RL1 L];
d1=2;%No Offset Along Prismatic Joint

Twist = [0 -pi/2 0 0 0 0 0 0 0 0];
%Offset= [d1 0 0 0 0 0 0 0 0 0];
LinkNo=length(ALL);

Links{1} = link([Twist(1) ALL(1) 0 0 1 d1], 'standard');%Here is Prismatic Joint
for ii=2:LinkNo %Make the links of the robot
    Links{ii} = link([Twist(ii) ALL(ii) 0 0], 'standard');
end
%plot3(0,0,0); hold on;
SnakeRobot = robot(Links);
SnakeRobot.name = 'SnakeRobot_T';
AngleSet=zeros(1,length(ALL));      %Set Initial Joint Angles
%dispRobot(SnakeRobot, AngleSet(1,1:LinkNo)); hold on;

TPos = [51.264 17.446 2];
% TPos = [8.6005	0.40728	5.355];
% TPos = [8.6001	-6.4919	1.5051];

PV_R1=[PL1 0 d1]; %Set Cartesian coordinates of the first revolute joint

dist_R1_mTpos = distBTW(PV_R1, TPos);%dist from PV_R1 to TPos, mTpos: Mirror of TPos
Def_End_Eff=[sum(ALL) 0 d1];
mVec=Def_End_Eff-PV_R1;
mTpos=PV_R1 + (mVec * (dist_R1_mTpos/distBTW(PV_R1, Def_End_Eff)));

%NB: Since first joint is prismatic hence its anglular value=0;
%Angle of elevation in 2nd DOF is calculated as 
phi = cosRule(PV_R1(1:2), mTpos(1:2), TPos(1:2)); 
AngleSet(1,2)=phi; %Update Value of 1st Revolute Joint  
%Now, We can obtain the Base point of the sub-links operating in 3rd DOF
cTPos=getDefaultLinkPosition(Twist, ALL, AngleSet, Links{1});%cTpos: Current TPos
%Vec=normzz(cTPos-PV_R1); %The Vector Was Normalized First
% PV_R2= [1/9 * abs(cTPos(1:2) + PV_R1(1:2)), PV_R1(3)]; %This computes PV_R2 along TPos from PV_R1 is 
%PV_R2=getDefaultLinkPosition(Twist(1:2), ALL(1:2), AngleSet(1:2), Links{1});
PV_R2= [PV_R1(1:2) + ((1/9) * (cTPos(1:2) - PV_R1(1:2))), PV_R1(3)];

%Attention is given to sign of y-axis as x-axis can only be positive
if(cTPos(2)<0) PV_R2(2)=PV_R2(2)*-1; end

%Now, we can virtualize points for other joints in the 3rd DOF.
%Before, convert the two end points to 2-dim
PV_R2_2D = [sqrt(PV_R2(1)^2 + PV_R2(2)^2), PV_R2(3)];
TPos_2D = [sqrt(TPos(1)^2 + TPos(2)^2), TPos(3)];

%Divide the planar line into 4 segements, and get coordinates of all links
Points=segPlane(L, PV_R2_2D, TPos_2D, 4);

for i=1:length(Points)-1 
  	AngleSet(1,i+2) = atan2(Points(i+1,2)-Points(i,2), Points(i+1,1)-Points(i,1));
end
dispRobot(SnakeRobot, AngleSet(1,1:LinkNo)); hold on;
for i=2:length(Points)-1    
     plot(Points(i,1), Points(i,2), 'go');      plot(Points(i,1), Points(i,2), 'g*');
end

RobPos=fkine(SnakeRobot, AngleSet(1,1:LinkNo));
NavError = minus(RobPos(1:3,4), TPos');

plot3(PV_R2(1), PV_R2(2), PV_R2(3), 'r*'); text(PV_R2(1), PV_R2(2), PV_R2(3),'R_2', 'Color', 'red','FontSize', 12);
plot3(PV_R1(1), PV_R1(2), PV_R1(3), 'r*'); text(PV_R1(1), PV_R1(2), PV_R1(3),'R_1', 'Color', 'red','FontSize', 12);
plot3(TPos(1), TPos(2), TPos(3), 'bo');     plot3(TPos(1), TPos(2), TPos(3), 'r*');
fprintf('Movement made in %3.2f Sec(s) has error %3.2f, %3.2f, %3.2f\n', toc(start_timer), NavError(1), NavError(2), NavError(3));  

%axis([-40 20 -20 60 0 30]);