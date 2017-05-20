function ErrorTest

    SEGNUM = 5;
    
    mcs_initialize( );

    %Fixit: SEGNUM should be used here
    [body1, body2, body3, body4, body5, body6] = GetBodyFromMCS( );%From tip to root
    [currentState] = ComputeState( body1, body2, body3, body4, body5, body6 );
    
    mcs_clear( );

    fliplr( currentState(:, 1) )  %for Error in x
    fliplr( currentState(:, 2) )  %for Error in y

    figure( 1 );
    plot( currentState(:,1), 'o' );
    title( 'ErrorinX' );

    figure( 2 );
    plot( currentState(:,2), 'o' );
    title( 'ErrorinY' );