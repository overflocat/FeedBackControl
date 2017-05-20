function [stateone] = deal_data_from_mcs_2body_task1( body1,body2)
[centre_root,root_x,root_y,root_z]=deal_body(body2);
[centre1,vx1,vy1,vz1]=deal_body(body1);
% centre1=centre1-0.105*vy1-0.12*vz1;
% centre1=centre1-0.03*vy1-0.115*vz1+0*vx1;   %jimu

centre1=centre1-0.09*vy1-0.1*vz1+0.02*vx1;   %  pingzhi 

[coordinate1,ph1,theta1] = deal_tran(centre_root,root_x,root_y,root_z,centre1,vz1);

stateone=[coordinate1*1000,ph1,theta1];
