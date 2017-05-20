function [deltaPinR, deltaPinDis] = GetDeltaPX( motion_source, motion_next, k_R, k_Dis, pNow )
    %J=[r r;x x];
    
    global SEGNUM;
    global MAXP;
    
    k_correct = 0.1; %Mistery const
    deltaPinR = zeros( SEGNUM, 1 );
	deltaPinDis = zeros( SEGNUM, 1 );

    [deltaR, deltaDis] = get_motion_delta( motion_source, motion_next );
    deltaR_temp = deltaR;
    deltaDis_temp = deltaDis;

	for i = 1:SEGNUM

        if ( deltaR_temp(i) * deltaR(i) < 0 )
            deltaR_temp(i) = k_correct * deltaR(i);
        end
        
        J = get_jacobian( motion_source, i, i, k_R, k_Dis );
        delta_p = J^-1 * [deltaDis_temp(i); deltaR_temp(i)];
        deltaPinDis(i) = delta_p(1);
        deltaPinR(i) = delta_p(2);
        
        max_p = min( [pNow(4*i-3) pNow(4*i-2) pNow(4*i-1) pNow(4*i)] );
        deltaPinDis(i) = max( deltaPinDis(i), -max_p );
        min_p = MAXP - max( [pNow(4*i-3) pNow(4*i-2) pNow(4*i-1) pNow(4*i)] );
        deltaPinDis(i) = min( deltaPinDis(i), min_p );
        
        deltaPinR(1) = 0;
        
        for j = i+1:SEGNUM
            J = get_jacobian( motion_source, i, j, k_R, k_Dis );
            delta_location = J * delta_p;
            deltaR_temp(j) = deltaR_temp(j) - delta_location(2)*(SEGNUM-(j-i))/SEGNUM;
            deltaDis_temp(j) = deltaDis_temp(j) - delta_location(1)*(SEGNUM-(j-i))/SEGNUM;
        end

    end

    deltaPinR = 0.9 * deltaPinR;
    deltaPinDis = 0.2 * deltaPinDis;

end