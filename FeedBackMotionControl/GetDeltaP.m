function [deltaPinR, deltaPinDis] = GetDeltaP( motion_source, motion_next, k_R, k_Dis, pNow )
    %J=[r r;x x];
    
    global SEGNUM;
    global MAXP;
    
    k_correct = 0.1; %Mistery const
    deltaPinR = zeros( SEGNUM, 1 );
	deltaPinDis = zeros( SEGNUM, 1 );

    [deltaR, deltaDis] = get_motion_delta( motion_source, motion_next );
    deltaR_temp = deltaR;
    deltaDis_temp = deltaDis;

    %Adjust the motion of arm from root to tip
	for i = 1:SEGNUM

        if ( deltaR_temp(i) * deltaR(i) < 0 )
            deltaR_temp(i) = k_correct * deltaR(i);
        end
        
        %Firstly, let the position of segment i reachs the position
        J = get_jacobian( motion_source, i, i, k_R, k_Dis );
        delta_p = J^-1 * [deltaDis_temp(i); deltaR_temp(i)];
        deltaPinDis(i) = delta_p(1);
        deltaPinR(i) = delta_p(2);
        
        max_p = min( [pNow(4*i-3) pNow(4*i-2) pNow(4*i-1) pNow(4*i)] );
        deltaPinDis(i) = max( deltaPinDis(i), -max_p );
        min_p = MAXP - max( [pNow(4*i-3) pNow(4*i-2) pNow(4*i-1) pNow(4*i)] );
        deltaPinDis(i) = min( deltaPinDis(i), min_p );
        
        %Due to the distribution of valves, the last segment( segment near the root ) can not bend
        deltaPinR(1) = 0;
        %Then add the influence of the adjustment to the rest segments and change deltaR&Dis
        for j = i+1:SEGNUM
            J = get_jacobian( motion_source, i, j, k_R, k_Dis );
            delta_location = J * delta_p;
            deltaR_temp(j) = deltaR_temp(j) - delta_location(2)*(SEGNUM-(j-i))/SEGNUM;
            deltaDis_temp(j) = deltaDis_temp(j) - delta_location(1)*(SEGNUM-(j-i))/SEGNUM;
        end

    end

    %I don't know the reason why there are two statments
    %Please ask Mr.Jin, leave it alone or just remove it
    deltaPinR = 0.9 * deltaPinR;
    deltaPinDis = 0.9 * deltaPinDis;

end