function circle(x,y,r, lineSpec)
    if(~exist('lineSpec','var'));   lineSpec='b';  end
    th = 0:pi/50:2*pi;
    xunit = r * cos(th)+x;
    yunit = r * sin(th)+y;
    %z=h * ones(1, length(t));%I added z to make 3D
    plot(xunit, yunit, lineSpec, 'LineWidth',1);