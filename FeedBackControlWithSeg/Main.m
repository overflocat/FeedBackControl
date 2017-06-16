function Main
    
    delete(instrfind);
    global s;
    s = serial('COM3', 'baudrate', 9600);
    fopen( s );

    pressure_for_16_05s( zeros(1, 16) );
    time = 30;
    k_p=[0.02,0.0004,0.0004,0.02,0.02];
        
    goal = [0,-160,350,0,0;
            0,-120,380,0,0;
            0,-60,390,0,0];
    [point_num, ~] = size(goal);

    p_now=zeros(1,16);
    pressure_for_16_05s(p_now);

    mcs_initialize();
    for i = 1:point_num
        [p_now,k_p] = goto_goal_3d_5seg( goal(i,:), time, p_now,k_p);
    end

    mcs_clear();
    pressure_for_16_05s( zeros(1, 16) );