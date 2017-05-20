function feedback_paper3_2

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
    time = 5;
    pause(5);
    k_p=[0.02,0.0004,0.0004,0.02,0.02];
        goal=[0,-10,340,0,0;
            0,-10,350,0,0;
            0,-10,360,0,0;
            0,-10,370,0,0;
            0,-10,380,0,0;
            0,-10,400,0,0;
            0,-10,410,0,0;
            0,-30,410,0,0;
            0,-50,410,0,0;
            0,-70,410,0,0;
            0,-90,410,0,0;
            0,-110,410,0,0;
            0,-120,410,0,0;
            0,-120,400,0,0;
            0,-120,390,0,0;
            0,-120,380,0,0;
            0,-120,370,0,0;
            0,-120,360,0,0;
            0,-120,350,0,0;
            0,-120,340,0,0;
            0,-100,340,0,0;
            0,-80,340,0,0;
            0,-60,340,0,0;
          0,-40,340,0,0;
          0,-20,340,0,0;
          0,-10,340,0,0;
            ];
    [point_num, ~] = size(goal);
    
     p_now=zeros(1,16);
     mcs_initialize();
    for j=1:5
        for i = 1:point_num
            [p_now,k_p] = goto_goal_3d_5seg_save_data( goal(i,:), time, p_now,k_p );
            p_now
        end
    end
    
    mcs_clear();
    pressure_for_16_05s( zeros(1, 16) );