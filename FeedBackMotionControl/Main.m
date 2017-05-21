function Main

    %Open Controller
    delete( instrfind );
    global s;
    s = serial( 'COM3', 'baudrate', 9600 );
    fopen( s );

    %Definition of values
    global SEGNUM; %The number of segments
    global MAXP; %The MAXIMUM of Pressure
    global DEBUGFLAG; %1 for DEBUG 
    SEGNUM = 5;
    MAXP = 3000;
    DEBUGFLAG = 1;
    ERRORRANGE = 0; %If Error < ERRORRANGE then break, its scale is mm
    TIMES = 20; %Iteration time for one motion
    pNow = zeros( 1, 4*SEGNUM ); %Current pressure
    K = [0.009, 0.06, 0.002, 0.06]; %k_dpdx, k_dpddis, k_dpdy, k_dpddis
%     
% K = [0.009, 0.06, 0.002, 0.06]*2;
%     motion = [    0  0   53.6323;
%    0 0  140;
%   0  0  220;
%   0  0  300;
%   0  0  380];
%     motion = [    -2.1059   -3.2569   53.6323;
%    -9.8920  -17.0373  141.3948;
%   -17.8105  -50.3920  238.8652;
%   -34.8795  -78.1039  319.7628;
%   -52.5440  -94.9787  404.5254];
    load('data2.mat','relative_coordinate');
    motion = relative_coordinate;

    motionNum = size( motion, 1 ) / SEGNUM;
    for i = 1 : motionNum
        motion( 5*i-4,3 ) = motion( 5*i-4,3 ) + 0;
        motion( 5*i-3,3 ) = motion( 5*i-3,3 ) + 4;
        motion( 5*i-2,3 ) = motion( 5*i-2,3 ) + 4;
        motion( 5*i-1,3 ) = motion( 5*i-1,3 ) + 4;
        motion( 5*i-0,3 ) = motion( 5*i-0,3 ) + 4;
    end
    motion(:,end)=motion(:,end)*1.2;

    Pressure( pNow );
    mcs_initialize( );

    for i = 1 : motionNum
        if( mod(i, 3) ~= 0 )
            continue;
        end
        [pNow, K] = ReachMotion( motion(i*5-4:i*5,:), TIMES, pNow, K, ERRORRANGE );
    end

    mcs_clear( );
    pNow = zeros( 1, 5*SEGNUM );
    
    Pressure( pNow );

end
