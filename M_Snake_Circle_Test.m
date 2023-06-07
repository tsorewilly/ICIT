clc;clear;
workspace=circle3([30, 9, 15], 5)';
% workspace =[436 760; 436 760; 451 743; 462 735; 478 721; 493 712; 513 702;532 696; 551 692; 568 693; 584 699; 595 712; 604 728; ...%3dof Character data-points
%     607 749; 609 771; 606 798; 601 824; 592 854; 580 882; 566 906; 550 927; 535 945; 520 957; 504 964; 491 969; 481 969; 481 969; 469 961; 469 961;...
%     469 945; 474 936; 483 927; 494 918; 509 912; 527 904; 544 900; 565 900; 584 906; 601 915; 616 932; 630 953; 642 978; 651 1005; 658 1032; 662 1059; ...
%     664 1085; 662 1107; 655 1126; 643 1141; 625 1153; 602 1162; 575 1171; 544 1174; 510 1177; 652 1035; 628 1036; 588 1047; 548 1054; 500 1070; 452 1083; ...
%     407 1099; 366 1117; 332 1135; 305 1158; 287 1177; 278 1197; 280 1212; 292 1227; 313 1236; 340 1244; 373 1245; 411 1237; 453 1222; 495 1201; 538 1174; ...
%     575 1141; 609 1102; 639 1054; 667 1002; 688 943; 703 886; 718 823; 726 764; 731 710; 732 660; 731 623; 728 598; 720 589; 720 589; 705 613; 701 641; ...
%     693 691; 693 744; 690 811; 691 878; 694 950; 700 1013; 707 1076; 720 1133; 734 1178; 749 1215; 773 1237; 796 1257; 826 1266; 858 1264; 889 1250; ...
%     924 1224; 400 972; 374 973; 344 983; 318 990; 290 1005; 267 1021; 247 1041; 234 1066; 226 1092; 229 1120; 238 1144; 253 1168; 279 1187; 312 1201;...
%     348 1211; 388 1213; 429 1209; 469 1197; 505 1182; 536 1160; 559 1135; 570 1107; 573 1076; 564 1046; 545 1017; 512 996; 471 980; 419 974; 363 973;...
%     645 442; 639 421; 632 411; 622 413; 608 435; 602 471; 587 537; 574 617; 557 721; 538 843; 516 990; 495 1148; 475 1308; 459 1462; 448 1606; ...
%     440 1731; 435 1836; 435 1915; 443 1958; 444 1982; 454 1978; 453 1967; 451 1934; 445 1897; 437 1852; 335 1531; 356 1515; 382 1508; 421 1499; 464 1495;...
%     513 1493; 575 1484; 634 1476; 694 1469; 758 1455; 820 1435]./100;       workspace(:,3) = 2; workspace(:,3) = randi(7,172,1)-1;
figure; plot3(workspace(:,1),workspace(:,2),workspace(:,3),'ob-','LineWidth',2); hold on;

start_timer=tic;
PL1=10; RL1=6.4;
L=[6.4 6.4 6.4 6.4 6.4 6.4 6.4 6.4];
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

Good=0;  Bad=0; Perf=0;  Threshold=1;    Comples=0; MoreBa=0; JustBa=0;
Lent=length(workspace);

for Id=1:54            %[29626 30330]
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
    NavError = minus(RobPos(1:3,4), TPos');
    
    Valid_S_4_2=[TPos RobPos(1:3,4)' NavError' AngleSet(2:LinkNo,1)'];

    if(~isreal(AngleSet))         
         Comples=Comples+1;
         ComplexPoint(Comples,:)=[Id Valid_S_4_2];
    else
        fprintf('%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f\t%3f%%\n', Valid_S_4_2,(Id/Lent)*100);pause(0.001);
        %disp([Valid_S_4_2 (Id/Lent)*100]);pause(0.001);
%         plot3(RobPos(1,4), RobPos(2,4), RobPos(3,4),'*r-','LineWidth',2);
%         if(sum(abs(NavError)<1)==3)
              Good=Good+1;         GoodPoint(Good,:)=[Id Valid_S_4_2];
%             
%             if(sum(abs(NavError)<Threshold)==3)
%                 Perf=Perf+1;     Perfect(Perf,:)=[Id Valid_S_4_2];
%                 plot3(RobPos(1,4), RobPos(2,4), RobPos(3,4),'*g','LineWidth',2);
%             elseif(sum(abs(NavError)<Threshold)==2)
%                 JustBa=JustBa+1; Almost(JustBa,:)=[Id Valid_S_4_2];
%                 plot3(RobPos(1,4), RobPos(2,4), RobPos(3,4),'*c','LineWidth',2);
%             else%if(sum(abs(NavError)<Threshold)==1)
%                 MoreBa=MoreBa+1; BitFar(MoreBa,:)=[Id Valid_S_4_2];
%                 plot3(RobPos(1,4), RobPos(2,4), RobPos(3,4),'*m','LineWidth',2);
%             end 
%         else
%             Bad=Bad+1;      BadPoint(Bad,:)=[Id Valid_S_4_2];
%             plot3(RobPos(1,4), RobPos(2,4), RobPos(3,4),'.r','LineWidth',1);
%         end               
    end
    vars2keep = {'start_timer','PL1','RL1','d1','LinkNo','Twist','L','ALL','SnakeRobot','Links','workspace','Threshold','Lent','Good','Bad','Comples','Perf','MoreBa','JustBa','GoodPoint', 'BadPoint','Perfect','Almost','BitFar','ComplexPoint'};    
%     clearvars('-except',vars2keep{:});    
end
plot3(GoodPoint(:,5), GoodPoint(:,6), GoodPoint(:,7),'*r-','LineWidth',2);
fprintf('Validation Completed in %f Sec(s)\n', toc(start_timer));
fprintf('Total \t\t\t Perfect \t\t Almost \t\t\t BitFar \t\t\t Bad \t\t\tAccuracy\n%d \t\t\t\t %d\t\t\t\t%d\t\t\t\t\t%d\t\t\t\t%d\t\t\t\t%f%%\n',Lent,Perf,JustBa,MoreBa,Bad,((Good/Lent)*100));
xlabel('X-Axis', 'fontsize',12, 'fontweight', 'bold');
ylabel('Y-Axis', 'fontsize',12, 'fontweight', 'bold');
zlabel('Z-Axis', 'fontsize',12, 'fontweight', 'bold');
% title({'\color{black} WORKSPACE FOR 4-LINK SNAKE ROBOT','\color{magenta}Generated By Triangular Geometry'},'fontsize',16);
