function [centre,x,y,z] = deal_body( body)
%centre表示坐标，x,y,z,表示以他自己建立的坐标系的轴的向量
y1=body(1,2);
y2=body(2,2);
y3=body(3,2);

yy=[y1,y2,y3];
tip=find(max(yy)==yy);
pointone=body(tip,:);
body(tip,:)=[];
pointtwo=(body(1,:)+body(2,:))/2;
centre=(pointone+pointtwo)/2;
y=-(pointtwo-pointone)/sqrt(sum((pointone-pointtwo).^2));
x=(body(1,:)-body(2,:))/sqrt(sum((body(1,:)-body(2,:)).^2));
z=cross(x,y);
if(z(1)<0)

    x=-x;
    z=cross(x,y);


end
