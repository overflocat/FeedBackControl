function feedback_for_video

    delete(instrfind);
    global s;
    s = serial('COM3', 'baudrate', 9600);
    fopen( s );
%     addpath('C:\Users\SOFTROBOT\Desktop\HPN_control_system\controller_interface');
%     %   kinect2 initialize
%     addpath('C:\Users\SOFTROBOT\Desktop\HPN_control_system\kinect2_interface');
%     addpath('C:\Users\SOFTROBOT\Desktop\HPN_control_system\kinect2_interface\Mex');
%     %   data process
%     addpath('C:\Users\SOFTROBOT\Desktop\HPN_control_system\kinematic_calculation');
%     addpath('C:\Users\SOFTROBOT\Desktop\HPN_control_system\mcs_interface');
    p_now=zeros(1, 16);
    
    pause(2)
    pressure_for_16_05s(p_now );
    
    goal2 =[ -57.7807 -132.9881  382.2315   -0.3161    0.0347]
    goal1 =goal2;
    goal1(2)=goal1(2)+30;
    times = 15;
    k_p=[0.02,0.0004,0.0004,0.02,0.02];
    mcs_initialize();
    [p_now,k_p]=goto_goal_3d_5seg_for_video(times,p_now,k_p)
    
    [body1, body2, body3] = mcs_date_3_body();
    [goal] = deal_data_from_mcs_2body_task1( body3,body2 );
    [p_now,k_p] = goto_goal_3d_5seg( goal, 10, p_now, k_p  );
%        a=[130 130 50 50 100 100 50 50 100 100 50 50 130 50 0 0]*10;
%        
%        pressure_for_16_05s(a );
%        p_now=a;
%     pause(5)
%                 [body1, body2] = mcs_date_2_body();
%             [goal] = deal_data_from_mcs_2body( body1,body2)
    p_now(15)=2000;
    pressure_for_16_05s(p_now );
    
    pause
    
    goal(2)=goal(2)+40;
    
    [p_now,k_p] = goto_goal_3d_5seg( goal, 10, p_now, k_p  );
    goal(2)=goal(2)-30;
    [p_now,k_p] = goto_goal_3d_5seg( goal, 20, p_now, k_p  );
    
    
    pause;
    [p_now,k_p] = goto_goal_3d_5seg( goal1, 10, p_now, k_p  );
    [p_now,k_p] = goto_goal_3d_5seg( goal2, 20, p_now, k_p  );
    
    
    
    p_now(15)=0;
    for n = 1: 14
        if(p_now(n) > 0)
            p_now(n) = fix(0.5*p_now(n));
        end
    end
    %         pause;
    pressure_for_16_05s(p_now);
    pause(2)
    
    

    mcs_clear();
    pressure_for_16_05s( zeros(1, 16) );
    
end

    
    
    
    
    
    
    
    
    
    
    
    
    
    