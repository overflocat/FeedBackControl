function feedback_paper1

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
       mcs_initialize();
       a=[100 100 50 50 100 100 50 50 100 100 50 50 130 50 0 0];
       state_list=[];
       b=a*20;
%        for i=1:20
%            p=a*i;
%            pressure_for_16_05s(p);
%        end
            [body1, body2] = mcs_date_2_body();
            [stateone] = deal_data_from_mcs_2body( body1,body2)
            state_list=[state_list;stateone];
            pressure_for_16_05s(b);
            pause(0.15)
        for i=1:10
           [body1, body2] = mcs_date_2_body();
%zhu yi IP
            [stateone] = deal_data_from_mcs_2body( body1,body2)
            state_list=[state_list;stateone];
            pause(0.7)
        end
       pause(5)
       b(15)=2000;
       pressure_for_16_05s(b);
       for i=1:10
           [body1, body2] = mcs_date_2_body();
%zhu yi IP
            [stateone] = deal_data_from_mcs_2body( body1,body2)
            state_list=[state_list;stateone];
            pause(0.7)
        end
       
       
       mcs_clear();
       pressure_for_16_05s( zeros(1, 16) );
       state_list
       save('paper_3_1_2','state_list')
end
    
    
    
    
    