function feedback_paper3

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
    time = 50;
    k_p=[0.02,0.0004,0.0004,0.02,0.02];
        goal=[
            
%             0,65,350,0,0;
%             20,65,350,0,0;
%             40,65,350,0,0;
%             60,65,350,0,0;
%             65,65,350,0,0;
%             65,60,350,0,0;
%             65,40,350,0,0;
%             65,20,350,0,0;
%             65,0,350,0,0;
%             65,-20,350,0,0;
%             65,-40,350,0,0;
%             65,-60,350,0,0;
%             65,-65,350,0,0;
%             60,-65,350,0,0;
%             40,-65,350,0,0;
%             20,-65,350,0,0;
%             0,-65,350,0,0;
%             -20,-65,350,0,0;
%             -40,-65,350,0,0;
%             -60,-65,350,0,0;
            -65,-65,340,0,0;
%             -65,-60,350,0,0;
%             -65,-40,350,0,0;
%             -65,-20,350,0,0;
%             -65,0,350,0,0;
%             -65,20,350,0,0;
%             -65,40,350,0,0;
%             -65,60,350,0,0;
%             -65,65,350,0,0;
%             -60,65,350,0,0;
%             -40,65,350,0,0;
%             -20,65,350,0,0;
            ];
        goal(:,2)=goal(:,2)-60;
    [point_num, ~] = size(goal);
%     p_now=zeros(2,4);
     p_now=zeros(1,16);
     mcs_initialize();
    for j=1:5
        for i = 1:point_num
            [p_now,k_p] = goto_goal_3d_5seg_save_data( goal(i,:), time, p_now,k_p );
            p_now
    %         [p_now] = FeedBackControl3D( p_now, goal(i,:), time );
    %         pause(4);
        end
    end

    mcs_clear();
    pressure_for_16_05s( zeros(1, 16) );