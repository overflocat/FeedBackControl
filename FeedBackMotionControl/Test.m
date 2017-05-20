function Test(  )

     mcs_initialize( );
     
     Pressure(zeros(20,1));

    %Fixit: SEGNUM should be used here
    [body1, body2, body3, body4, body5, body6] = GetBodyFromMCS( );%From tip to root
    [currentState_1] = ComputeState( body1, body2, body3, body4, body5, body6 );
    
    Pressure([1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]*3000);
    
    [body1, body2, body3, body4, body5, body6] = GetBodyFromMCS( );%From tip to root
    [currentState_2] = ComputeState( body1, body2, body3, body4, body5, body6 );
    
    a = currentState_1(5,3) - currentState_1(4,3)
    b = currentState_2(5,3) - currentState_2(4,3)
    
    b / a
    
    
end

