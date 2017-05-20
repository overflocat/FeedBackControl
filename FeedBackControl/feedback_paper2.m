function feedback_paper2

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
    time = 40;
    k_p=[0.02,0.0004,0.0004,0.02,0.02];
        goal=[0,-35,380,0,0;
            90,-120,350,0,0;
            -90,-120,350,0,0;
            
            ];
    [point_num, ~] = size(goal);
%     p_now=zeros(2,4);
     p_now=zeros(1,16);
          p_now(15)=2000;
     pressure_for_16_05s( p_now );
     mcs_initialize();
    for j=1:3
        for i = 1:point_num
            [p_now,k_p] = goto_goal_3d_5seg_save_data( goal(i,:), time, p_now,k_p );
            p_now
    %         [p_now] = FeedBackControl3D( p_now, goal(i,:), time );
    %         pause(4);
        end
    end

    mcs_clear();
    pressure_for_16_05s( zeros(1, 16) );