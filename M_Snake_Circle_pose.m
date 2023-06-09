workspace=circle3([30, 9, 15], 5)';
start_timer=tic;
PL1=100; RL1=64;
L=[64 64 64 64 64 64 64 64];
ALL=[PL1 RL1 L];
d1=2;%No Offset Along Prismatic Joint

Twist = [0 -pi/2 0 0 0 0 0 0 0 0];
%Offset= [d1 0 0 0 0 0 0 0 0 0];
LinkNo=length(ALL);

Links{1} = link([Twist(1) ALL(1) 0 0 1 d1], 'standard');%Here is Prismatic Joint
for ii=2:LinkNo %Make the links of the robot
    Links{ii} = link([Twist(ii) ALL(ii) 0 0], 'standard');
end
SnakeRobot = robot(Links);
SnakeRobot.name = 'SnakeRobot_T';

Lent=length(workspace);

for Id=1:Lent            %[29626 30330]
    TPos=workspace(Id, 1:3);
    AngleSet=zeros(length(ALL),1);      %Reset Initial Joint Angles

    PV_R1=[PL1 0 d1]; %Set Cartesian coordinates of the first revolute joint
    
    dist_R1_mTpos = distBTW(PV_R1, TPos);%dist from PV_R1 to TPos, mTpos: Mirror of TPos
    Def_End_Eff=[sum(ALL) 0 d1];
    mVec=Def_End_Eff-PV_R1;
    mTpos=PV_R1 + (dist_R1_mTpos/distBTW(Def_End_Eff, PV_R1)) * mVec;
    pTpos=[TPos(1) TPos(2) mTpos(3)];%TPos on XY plane
    %NB: Since first joint is prismatic hence its anglular value=0;
    %Angle of elevation in 2nd DOF is calculated as 
    phi=GetAngle(mTpos, pTpos, PV_R1);

    AngleSet(2,1)=phi; %Update Value of 1st Revolute Joint 

    %Now, We can obtain the Base point of the sub-links operating in 3rd DOF
    cTPos=getDefaultLinkPosition(Twist, ALL, AngleSet, Links{1});%cTpos: Current TPos
    %Vec=normzz(cTPos-PV_R1); %The Vector Was Normalized First
    PV_R2= [PV_R1(1:2) + ((1/9) * (cTPos(1:2) - PV_R1(1:2))), PV_R1(3)];
    ratio=distBTW(PV_R2, TPos)/distBTW(PV_R2, cTPos);
    rTPos=[PV_R1(1:2) + (ratio * (cTPos(1:2) - PV_R2(1:2))), PV_R2(3)];
    %Attention is given to sign of y-axis as x-axis can only be positive
    if(cTPos(2)<0) PV_R2(2)=PV_R2(2)*-1; end

    % radius=distBTW(PV_R2, TPos); 
    EP1= [sqrt(PV_R2(1)^2+PV_R2(2)^2) PV_R2(3)];    EP2=[sqrt(TPos(1)^2+TPos(2)^2), TPos(3)];
    dimLossRatio=(distBTW(PV_R2, TPos)-distBTW(EP1, EP2))/distBTW(EP1, EP2);
    EP2=EP2 + (dimLossRatio * (EP2 - EP1));

    global sLine; sLine='n o';
                    V4=getIntCoords(EP1, EP2, sum(L)/2, 4);
                    V2=getIntCoords(EP1, V4, sum(L)/4, 2);
    sLine='yes';    V1=getIntCoords(EP1, V2, sum(L)/8, 1);
                    V3=getIntCoords(V2, V4, sum(L)/8, 3);
    sLine='n o';    V6=getIntCoords(V4, EP2, sum(L)/4, 6);
    sLine='yes';    V5=getIntCoords(V4, V6, sum(L)/8, 5);
                    V7=getIntCoords(V6, EP2, sum(L)/8, 7);
                    
    Points={EP1; V1; V2; V3; V4; V5; V6; V7; EP2};   
    defRatio=L(1)/distBTW(EP1, EP2); 
    defVec=EP1 + (defRatio * (EP2 - EP1));

    for ii=2:length(Points) 
        AngleSet(ii+1, 1)=GetAngle(defVec, Points{ii}, Points{ii-1});
        AngleSet(ii+1, 2)=distBTW(Points{ii}, Points{ii-1});
        if ii<length(Points); 
            defRatio=L(ii)/distBTW(Points{ii}, Points{ii+1});
            defVec=Points{ii} + (defRatio * (Points{ii}-Points{ii-1}));
        end
    end
    %Update Joint At R2 Base On TPos
    AngleSet(3, 1)=0;

    RobPos=fkine(SnakeRobot, AngleSet(1:LinkNo,1));     RobPos=RobPos(1:3,4)';
    Curr=[sqrt(RobPos(1)^2 + RobPos(2)^2) RobPos(3)];
    AngleSet(3, 1)=GetAngle(Curr, EP2, EP1);%Updated Value of Theta at Joint R2

    %Check If Sign if Correct
    CoSignAng=getDefaultLinkPosition(Twist, ALL, [AngleSet([1 2],1); -AngleSet(3, 1); AngleSet(4:end,1)], Links{1});
    OrSignAng=getDefaultLinkPosition(Twist, ALL, AngleSet, Links{1});

    if(distBTW(CoSignAng, TPos)<distBTW(OrSignAng, TPos))
        AngleSet(3, 1)=-AngleSet(3, 1);
    end

    RobPos=fkine(SnakeRobot, AngleSet(1:LinkNo,1));
    plot3(RobPos(1,4), RobPos(2,4), RobPos(3,4),'-r','LineWidth',2);
    plot(SnakeRobot, AngleSet');
    
    vars2keep = {'start_timer','PL1','RL1','d1','LinkNo','Twist','L','ALL','SnakeRobot','Links','workspace'};    
    clearvars('-except',vars2keep{:});    
end
fprintf('Validation Completed in %f Sec(s)\n', toc(start_timer));
xlabel('X-Axis', 'fontsize',12, 'fontweight', 'bold');
ylabel('Y-Axis', 'fontsize',12, 'fontweight', 'bold');
zlabel('Z-Axis', 'fontsize',12, 'fontweight', 'bold');
% title({'\color{black} WORKSPACE FOR 4-LINK SNAKE ROBOT','\color{magenta}Generated By Triangular Geometry'},'fontsize',16);
