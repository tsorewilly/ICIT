%%%%%%%%%%%%%%%%%%%%%%%Electrogilces.Inc%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Correction 
%Ex: No. 10 - Robot PRRR 4GDL
%%% First, we clean the workspace
clear
clc
%%
%%% QUESTION 1:
%%% Link generation
L(1) = Link([pi/2, 0, 100, 0, 1]);
L(2) = Link([ 0, 0, 100, 0]);
L(3) = Link([ 0, 0, 100, 0]);
L(4) = Link([ 0, 0, 0, 0]);
L(1).offset = -60;

%%% Robot generation
PRRR = SerialLink(L);
PRRR.name = 'PRRR';

%%% We plot the robot to verify that we have correctly generated it.
PRRR.plot([0 0 0 0])
%%
%%% QUESTION 2:
%%% We define the initial and final hand poses
TINI = transl(0,200,0);
TEND = transl(50,50,200)*trotz(pi/2);

%%% Before generating the trajectory, we can try to obtain the joint
%%% angles in the initial configuration using the ikine function
q0 = PRRR.ikine(TINI, [0 0 0 0], [1 1 1 0 0 1]);

%%% It does not converge to a solution. We have to change the initial
%%% guess but, after some trial and error, we observe that it is
%%% difficult to obtain a good initial guess. Thus, to overcome this
%%% problem we can use the function implemented in other file ikine3r
%%% 
T01INI = trotz(pi/2)*transl(100,0,0);
T01END = trotz(pi/2)*transl(100,0,200);
T14INI = inv(T01INI)*TINI;
T14END = inv(T01END)*TEND;
samples=100;

%%% TRAJECTORY 1

qini1 = [T01INI(3,4) ikine3r([100 100 0],...
[T14INI(1,4) T14INI(2,4) atan2(T14INI(1,2),T14INI(1,1))],-1)];
qend1 = [T01END(3,4) ikine3r([100 100 0],...
 [T14END(1,4) T14END(2,4) atan2(T14END(1,2),T14END(1,1))],-1)];
Q1 = jtraj(qini1, qend1, samples);
TRAJ1 = PRRR.fkine(Q1);
for i=1:samples
 xx1(i) = TRAJ1(1,4,i);
 yy1(i) = TRAJ1(2,4,i);
 zz1(i) = TRAJ1(3,4,i);
end
%%
%%% TRAJECTORY 2

qini2 = [T01INI(3,4) ikine3r([100 100 0],...
 [T14INI(1,4) T14INI(2,4) atan2(T14INI(1,2),T14INI(1,1))],1)];
qend2 = [T01END(3,4) ikine3r([100 100 0],...
 [T14END(1,4) T14END(2,4) atan2(T14END(1,2),T14END(1,1))],1)];
Q2 = jtraj(qini2, qend2, samples);
TRAJ2 = PRRR.fkine(Q2);
for i=1:samples
 xx2(i) = TRAJ2(1,4,i);
 yy2(i) = TRAJ2(2,4,i);
 zz2(i) = TRAJ2(3,4,i);
end
%%
%%% TRAJECTORY 3

qini3 = [T01INI(3,4) ikine3r([100 100 0],...
 [T14INI(1,4) T14INI(2,4) atan2(T14INI(1,2),T14INI(1,1))],1)];
qend3 = [T01END(3,4) ikine3r([100 100 0],...
 [T14END(1,4) T14END(2,4) atan2(T14END(1,2),T14END(1,1))],-1)];
Q3 = jtraj(qini3, qend3, samples);
TRAJ3 = PRRR.fkine(Q3);
for i=1:samples
 xx3(i) = TRAJ3(1,4,i);
 yy3(i) = TRAJ3(2,4,i);
 zz3(i) = TRAJ3(3,4,i);
end
%%
%%% TRAJECTORY 4

qini4 = [T01INI(3,4) ikine3r([100 100 0],...
 [T14INI(1,4) T14INI(2,4) atan2(T14INI(1,2),T14INI(1,1))],-1)];
qend4 = [T01END(3,4) ikine3r([100 100 0],...
 [T14END(1,4) T14END(2,4) atan2(T14END(1,2),T14END(1,1))],1)];
Q4 = jtraj(qini4, qend4, samples);
TRAJ4 = PRRR.fkine(Q4);
for i=1:samples
 xx4(i) = TRAJ4(1,4,i);
 yy4(i) = TRAJ4(2,4,i);
 zz4(i) = TRAJ4(3,4,i);
end
hold on;
plot3(xx1, yy1, zz1, 'Color', [1 0 0], 'LineWidth',2);
plot3(xx2, yy2, zz2, 'Color', [0 1 0], 'LineWidth',2);
plot3(xx3, yy3, zz3, 'Color', [0 0 1], 'LineWidth',2);
plot3(xx4, yy4, zz4, 'Color', [1 1 0], 'LineWidth',2);
PRRR.plot(Q1);
PRRR.plot(Q2);
PRRR.plot(Q3);
PRRR.plot(Q4);
%%
%%% QUESTION 3:
TC = ctraj(TINI, TEND, samples);
for i=1:samples
 T01(:,:,i) = trotz(pi/2)*transl(100,0,TC(3,4,i));
 T14(:,:,i) = inv(T01(:,:,i))*TC(:,:,i);
 QTRAJ1(i,:) = [T01(3,4,i) ...
 ikine3r([100 100 0], ...
 [T14(1,4,i) T14(2,4,i) atan2(T14(1,2,i),T14(1,1,i))],-1)];
 QTRAJ2(i,:) = [T01(3,4,i) ...
 ikine3r([100 100 0], ...
 [T14(1,4,i) T14(2,4,i) atan2(T14(1,2,i),T14(1,1,i))],1)];
end
TRAJ5 = PRRR.fkine(QTRAJ1);
TRAJ6 = PRRR.fkine(QTRAJ2);
for i=1:samples
 xx5(i) = TRAJ5(1,4,i);
 yy5(i) = TRAJ5(2,4,i);
 zz5(i) = TRAJ5(3,4,i);
 xx6(i) = TRAJ6(1,4,i);
 yy6(i) = TRAJ6(2,4,i);
 zz6(i) = TRAJ6(3,4,i);
end
hold on;
plot3(xx5, yy5, zz5, 'Color', [0 1 1], 'LineWidth',2);
plot3(xx6, yy6, zz6, 'Color', [1 0 1], 'LineWidth',2);
PRRR.plot(QTRAJ1);
PRRR.plot(QTRAJ2);