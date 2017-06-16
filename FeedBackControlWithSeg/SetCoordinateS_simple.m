function [UvectorDis,UvectorX] = SetCoordinateS_simple( x, y )
vdis = [x;y];
vx = [vdis(2); -vdis(1)];
length = sqrt( sum(vdis.^2) );

UvectorDis = vdis / length;
UvectorX = vx / length;