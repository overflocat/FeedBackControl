function [currentState] = ComputeState( body1, body2, body3, body4, body5, body6 )

    ERRORX_1 = 15 * 0.001;
    ERRORY_1 = 36 * 0.001;

    ERRORX_2 = 14 * 0.001;
    ERRORY_2 = 33 * 0.001;

    ERRORX_3 = -4 * 0.001;
    ERRORY_3 = 42 * 0.001;

    ERRORX_4 = 0 * 0.001;
    ERRORY_4 = 48 * 0.001;

    ERRORX_5 = 2 * 0.001;
    ERRORY_5 = 57 * 0.001;

    [mCenter, mX, mY, mZ] = deal_body_min( body6 ); %SetMainCoordinateSystem
    [center1, vx1, vy1, vz1] = deal_body( body1 ); %vz is ignored
    [center2, vx2, vy2, vz2] = deal_body( body2 );
    [center3, vx3, vy3, vz3] = deal_body( body3 );
    [center4, vx4, vy4, vz4] = deal_body( body4 );
    [center5, vx5, vy5, vz5] = deal_body( body5 );
    
    mCenter = mCenter -0.01 * mZ;
    center1 = center1 - ERRORX_1 * vx2 - ERRORY_1 * vy2;
    center2 = center2 - ERRORX_2 * vx2 - ERRORY_2 * vy2;
    center3 = center3 - ERRORX_3 * vx3 - ERRORY_3 * vy3;
    center4 = center4 - ERRORX_4 * vx4 - ERRORY_4 * vy4;
    center5 = center5 - ERRORX_5 * vx5 - ERRORY_5 * vy5;

    [coordinate1, ph1, theta1] = deal_tran( mCenter, mX, mY, mZ, center1, vz1 );
    [coordinate2, ph2, theta2] = deal_tran( mCenter, mX, mY, mZ, center2, vz2 );
    [coordinate3, ph3, theta3] = deal_tran( mCenter, mX, mY, mZ, center3, vz3 );
    [coordinate4, ph4, theta4] = deal_tran( mCenter, mX, mY, mZ, center4, vz4 );
    [coordinate5, ph5, theta5] = deal_tran( mCenter, mX, mY, mZ, center5, vz5 );    

    currentState = 1000 * [coordinate5, ph5, theta5; coordinate4, ph4, theta4; coordinate3, ph3, theta3; coordinate2, ph2, theta2; coordinate1, ph1, theta1];
