function [stateone] = deal_data_from_mcs_2body( body1,body2)
[centre_root,root_x,root_y,root_z]=deal_body(body2);
[centre1,vx1,vy1,vz1]=deal_body(body1);

[coordinate1,ph1,theta1] = deal_tran(centre_root,root_x,root_y,root_z,centre1,vz1);

stateone=[coordinate1*1000,ph1,theta1];
