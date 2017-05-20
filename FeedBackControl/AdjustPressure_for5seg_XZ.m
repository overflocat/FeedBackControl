function [p_temp] = AdjustPressure_for5seg_XZ ( dphdest, ddisdest, dxdest, k_dpdph, k_dpddis, k_dpdx,p_now )
    p_temp = zeros(1,16);
    dp_ph = dphdest / k_dpdph;
    dp_phs(1)=dp_ph*12 / 128;
    dp_phs(2)=dp_ph*3 / 64;
    dp_phs(3)=dp_ph / 64; 
%     max_p=min(p_now(2*i-1),p_now(2*i));
%     delta_p_dis(i)=max(delta_p_dis(i),-2*max_p);
%     min_p=2500-max(p_now(2*i-1),p_now(2*i));
%     delta_p_dis(i)=min(delta_p_dis(i),min_p*2);
    p_temp(1) =  p_temp(1) - dp_phs(1);
    p_temp(2)=  p_temp(2) + dp_phs(1);
    p_temp(3)=  p_temp(3) - dp_phs(1);
    p_temp(4)=  p_temp(4) + dp_phs(1);
    p_temp(5)=  p_temp(5) - dp_phs(2);
    p_temp(6) = p_temp(6) + dp_phs(2);
    p_temp(7) = p_temp(7) - dp_phs(2);
    p_temp(8) = p_temp(8) + dp_phs(2);
    p_temp(9)=  p_temp(9) - dp_phs(3);
    p_temp(10) =  p_temp(10) + dp_phs(3);
    p_temp(11) = p_temp(11) - dp_phs(3);
    p_temp(12) = p_temp(12) + dp_phs(3);
    
    

    dp_dis = ddisdest / k_dpddis;   %no need to adjust dis by theta because sum(p_temp) = 0
    dp_diss(1)=dp_dis / 20;
    dp_diss(2)=dp_dis / 20;
    dp_diss(3)=dp_dis / 20;
    p_max=2500;
    for i=1:3
        max_p=min([p_now(4*i-3),p_now(4*i-2),p_now(4*i-1),p_now(4*i)]);
        if i==3
            p_max=3500;
        end
        min_p=p_max-max([p_now(4*i-3),p_now(4*i-2),p_now(4*i-1),p_now(4*i)]);
        dp_diss(i)=max(dp_diss(i),-max_p);
        dp_diss(i)=min(dp_diss(i),min_p);
        
        p_temp(4*i-3)=p_temp(4*i-3)+dp_diss(i);
        p_temp(4*i-2)=p_temp(4*i-2)+dp_diss(i);
        p_temp(4*i-1)=p_temp(4*i-1)+dp_diss(i);
        p_temp(4*i)=p_temp(4*i)+dp_diss(i);   
    end
    
    
    difinx =(4*(p_temp(2)+p_temp(4)-p_temp(1)-p_temp(3))+ 5*(p_temp(6)+p_temp(8)-p_temp(5)-p_temp(7)) + 7*(p_temp(10)+p_temp(12)-p_temp(9)-p_temp(11)) ) * k_dpdx;       %注意现在的正负的意义变了
    dp_x = ( dxdest - difinx ) / k_dpdx;
    dp_x1=dp_x / 64;
    p_temp(1) =  p_temp(1) - dp_x1;
    p_temp(2)=  p_temp(2) + dp_x1;
    p_temp(3)=  p_temp(3) - dp_x1;
    p_temp(4)=  p_temp(4) + dp_x1;
    p_temp(5)=  p_temp(5) - dp_x1;
    p_temp(6) =  p_temp(6) + dp_x1;
    p_temp(7) = p_temp(7) - dp_x1;
    p_temp(8) = p_temp(8) + dp_x1;
    p_temp(9)=  p_temp(9) - dp_x1;
    p_temp(10) =  p_temp(10) + dp_x1;
    p_temp(11) = p_temp(11) - dp_x1;
    p_temp(12) = p_temp(12) + dp_x1;
    p_temp(15)=0;
    p_temp(16)=0;
