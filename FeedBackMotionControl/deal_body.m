function [centre,x,y,z] = deal_body( body )
%centre表示坐标，x,y,z,表示以他自己建立的坐标系的轴的向量
dis1_2=sqrt(sum((body(1,:)-body(2,:)).^2));
dis1_3=sqrt(sum((body(1,:)-body(3,:)).^2));
dis2_3=sqrt(sum((body(2,:)-body(3,:)).^2));

dis=[dis2_3,dis1_3,dis1_2];
tip=find(max(dis)==dis);
pointone=body(tip,:);
body(tip,:)=[];
pointtwo=(body(1,:)+body(2,:))/2;
centre=(pointone+pointtwo)/2;
y=(pointone-pointtwo)/sqrt(sum((pointone-pointtwo).^2));
if(y(2)<0)
    y = -y;
end
x=(body(1,:)-body(2,:))/sqrt(sum((body(1,:)-body(2,:)).^2));
z=cross(x,y);
if(z(1)<0)

    x=-x;
   z=cross(x,y);


end
