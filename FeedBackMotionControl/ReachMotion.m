function [pNow, K] = ReachMotion( motionGoal, TIMES, pNow, K, ERRORRANGE )

    global SEGNUM; %The number of segments
    global MAXP; %The MAXIMUM of Pressure
    global DEBUGFLAG; %1 for DEBUG 

    MAXPVAR = 200; %The maximum of pressure varitation in one iteration

    k_X = K(1);
    k_disX = K(2);
    k_Y = K(3);
    k_disY = K(4);

    for i = 1:TIMES

        %Fixit: SEGNUM should be used here
        [body1, body2, body3, body4, body5, body6] = GetBodyFromMCS( );%From tip to root
        [currentState] = ComputeState( body1, body2, body3, body4, body5, body6 );

        if( DEBUGFLAG == 1 )
            currentState( SEGNUM, : )
        end

        %Check ERROR, if ERROR < ERRORRANGE then break
        [deltaX, deltaDisX] = get_motion_delta( currentState(:,1:2:3), motionGoal(:,1:2:3) );
        [deltaY, deltaDisY] = get_motion_delta( currentState(:,2:1:3), motionGoal(:,2:1:3) );
        
        if max(deltaX(3:2:5))<20&&max(deltaY(3:2:5))<20
            break;
        end
        
        if( DEBUGFLAG == 1 )
            fprintf('\ndeltaX:');
            deltaX'
            fprintf('\ndeltaY:');
            deltaY'
            fprintf('\ndeltaDisX:');
            deltaDisX'
            fprintf('\ndeltaDisY:');
            deltaDisY'
        end
%         if (deltaX(5))<20&&(deltaY(5))<20
%             break;
%         end
        
        errorinX = max ( max( abs(deltaX) ), max( abs(deltaDisX) ) );
        errorinY = max ( max( abs(deltaY) ), max( abs(deltaDisY) ) );
        if( max( errorinX, errorinY ) < ERRORRANGE )
            break;
        end

        [deltaPinX, deltaPinDisX] = GetDeltaPX( currentState(:,1:2:3), motionGoal(:,1:2:3) ,k_X, k_disX, pNow );
        [deltaPinY, deltaPinDisY] = GetDeltaPY( currentState(:,2:1:3), motionGoal(:,2:1:3) ,k_Y, k_disY, pNow );
        
        deltaPinX = flipud( deltaPinX );
        deltaPinY = flipud( deltaPinY );
        deltaPinDisX = flipud( deltaPinDisX );
        deltaPinDisY = flipud( deltaPinDisY );
        
        if( DEBUGFLAG == 1 )
            deltaPinY';
            deltaPinDisY';
        end

        deltaP = zeros( 1, 4*SEGNUM );
        for j = 1:SEGNUM
            deltaP(j*4-3) = -deltaPinX(j) + deltaPinY(j);
            deltaP(j*4-2) = deltaPinX(j) + deltaPinY(j);
            deltaP(j*4-1) = -deltaPinX(j) + -deltaPinY(j);
            deltaP(j*4-0) = deltaPinX(j) + -deltaPinY(j);
            deltaP(j*4-3) = deltaP(j*4-3) + 0.5 * ( deltaPinDisX(j) + deltaPinDisY(j) );
            deltaP(j*4-2) = deltaP(j*4-2) + 0.5 * ( deltaPinDisX(j) + deltaPinDisY(j) );
            deltaP(j*4-1) = deltaP(j*4-1) + 0.5 * ( deltaPinDisX(j) + deltaPinDisY(j) );
            deltaP(j*4-0) = deltaP(j*4-0) + 0.5 * ( deltaPinDisX(j) + deltaPinDisY(j) );
        end

        %The variation of pressure for one airbag in one iteration can't be bigger than MAXPVAR
        for j = 1:SEGNUM*4
            if( deltaP(j) < -MAXPVAR )
                deltaP(j) = -MAXPVAR;
            end
            if( deltaP(j) > MAXPVAR )
                deltaP(j) = MAXPVAR;
            end
        end

        pNow = pNow + deltaP;
        pNow = fix( pNow );

        for j = 1:SEGNUM*4
%             if ( j <= 8 && pNow(j) > 2 * MAXP )
%                 pNow(j) = 2 * MAXP;
%             end
%             if( j > 8 && pNow(j) > MAXP )
%                 pNow(j) = MAXP;
%             end
            if( pNow(j) > MAXP )
                pNow(j) = MAXP;
            end
            if( pNow(j) < 0 )
                pNow(j) = 0;
            end
        end
        
        if( DEBUGFLAG == 1 )
            pNow
        end

        Pressure( pNow );
        
        pNow(1) = ( pNow(1) + pNow(5) ) / 2;
        pNow(2) = ( pNow(2) + pNow(6) ) / 2;
        pNow(3) = ( pNow(3) + pNow(7) ) / 2;
        pNow(4) = ( pNow(4) + pNow(8) ) / 2;
        pNow(5) = pNow(1);
        pNow(6) = pNow(2);
        pNow(7) = pNow(3);
        pNow(8) = pNow(4);

    end

end