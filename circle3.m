function [newPnts]=circle3(ptCoord,r, ~)
    if(~exist('colo','var'));      colo=rColor();    end
%     plot3(ptCoord(1), ptCoord(2), ptCoord(3), [lineSpec, '*']); hold on;
    % Original points, original plane
    t = linspace(0,2*pi,100);
    x = r*cos(t)+ptCoord(1);
    y = r*sin(t)+ptCoord(2);
    z = 0*t+ptCoord(3);
    pnts = [x;y;z];
    % unit normal for original plane
    n0 = [0;0;1]; 
    n0 = n0/norm(n0); 

    %Then, I would rotate the circle data into a new plane
    % unit normal for plane to rotate into plane is orthogonal to n1
    % given by equation: n1(1)*x + n1(2)*y + n1(3)*z = 0
    n1 = [1;1;1]; 
    n1 = n1/norm(n1); 
    % theta is the angle between normals
    costheta = dot(n0,n1) / ( norm(n0)*norm(n1) ); % cos(theta)
    sintheta = sqrt(1-costheta*costheta);                        % sin(theta)
    u = cross(n0,n1) / ( norm(n0)*norm(n1) ); % rotation axis...
    u = u/norm(u); % ... as unit vector
    C = 1-costheta;
    % the rotation matrix
    R=[u(1)^2*C+costheta, u(1)*u(2)*C-u(3)*sintheta, u(1)*u(3)*C+u(2)*sintheta
      u(2)*u(1)*C+u(3)*sintheta, u(2)^2*C+costheta, u(2)*u(3)*C-u(1)*sintheta
      u(3)*u(1)*C-u(2)*sintheta, u(3)*u(2)*C+u(1)*sintheta, u(3)^2*C+costheta];

    % Rotated points
    newPnts = R*pnts; %This rotates the circle
    %newPnts = pnts;

    %Visualize:
    plot3(newPnts(1,:),newPnts(2,:),newPnts(3,:),[colo,'-'],'LineWidth',1);
    
end