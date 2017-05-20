function feedback_task1

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
    pressure_for_16_05s( zeros(1, 16) );
    time = 20;
    k_p=[0.02,0.0004,0.0004,0.02,0.02];
    mcs_initialize();
    [body1, body2, body3] = mcs_date_3_body();
    [goal] = deal_data_from_mcs_2body_task1( body3,body2)
    goal0=goal;
       goal0(3)=goal0(3)-30;
       goal2=goal;
       goal2(2)=goal2(2)+100;
       goal3=goal;
       goal3(2)=goal3(2)-10;
        goal=[
            goal0;
            goal;
            goal2;
            goal3;
           ];
    [point_num, ~] = size(goal);
%     p_now=zeros(2,4);
     p_now=zeros(1,16);
        
    for i = 1:point_num
        [p_now,k_p] = goto_goal_3d_5seg( goal(i,:), time, p_now,k_p);
        p_now(15)=2000;
%         pause;
        pressure_for_16_05s(p_now);
        alpha=0.2;
%         [p_now] = FeedBackControl3D( p_now, goal(i,:), time );
%         pause(4);
    end
     p_now(15)=0;
        for n = 1: 14
           if(p_now(n) > 0)
               p_now(n) = fix(0.5*p_now(n));
           end
        end
%         pause;
        pressure_for_16_05s(p_now);
        
        pause(5);
    mcs_clear();
    pressure_for_16_05s( zeros(1, 16) );