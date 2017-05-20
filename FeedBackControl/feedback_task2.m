function feedback_task1

    delete(instrfind);
    global s;
    s = serial('COM3', 'baudrate', 9600);
    fopen( s );

    pressure_for_16_05s( zeros(1, 16) );
    time = 3;
    k_p=[0.02,0.0004,0.0004,0.02,0.02];
    mcs_initialize();
    p_now=zeros(1,16);

    for i = 1 : 10
        [body1, body2, body3] = mcs_date_3_body();
        [goal] = deal_data_from_mcs_2body_task2( body3,body2 );
        [p_now,k_p] = goto_goal_3d_5seg( goal, time, p_now, k_p  );
    end

    time = 5;
    for i = 1 : 4
        [body1, body2, body3] = mcs_date_3_body();
        [goal] = deal_data_from_mcs_2body_task1( body3,body2 );
        [p_now,k_p] = goto_goal_3d_5seg( goal, time, p_now, k_p  );
    end

    p_now(15)=2000;
    pressure_for_16_05s(p_now);
     
    time = 15;
    goal(2)=goal(2)+30;
    [p_now,k_p] = goto_goal_3d_5seg( goal, time, p_now, k_p  );
    
    goal2 =[36.4505 ,-196.2825 , 279.8213  , 0.0272   ,0.0098];
    goal3=goal2;
    goal3(2)=goal3(2)+50;
    goal4=goal2;
    goal4(2)=goal4(2)+10;
    
    time = 25;
    [p_now,k_p] = goto_goal_3d_5seg( goal3, time, p_now, k_p  );
    [p_now,k_p] = goto_goal_3d_5seg( goal4, time, p_now, k_p  );
    
    time = 10;
    [p_now,k_p] = goto_goal_3d_5seg( goal2, time, p_now, k_p  );
    p_now(15)=0;
    
    pressure_for_16_05s( p_now );
    mcs_clear();
    pressure_for_16_05s( zeros(1, 16) );