function [p_temp] = AdjustPressure_for5seg_YZ ( dthetadest, ddisdest, dxdest, k_dpdtheta, k_dpddis, k_dpdy,p_now )
    p_temp = zeros(1,16);
    dp_theta = dthetadest / k_dpdtheta;
    dp_theta1=dp_theta*8 / 100;
    dp_theta2=dp_theta*5 / 100;
    dp_theta3=dp_theta*3 / 100;
    dp_theta4=dp_theta*1 / 100;
    p_temp(1) =  p_temp(1) + dp_theta1;
    p_temp(2)=  p_temp(2) + dp_theta1;
    p_temp(3)=  p_temp(3) - dp_theta1;
    p_temp(4)=  p_temp(4) - dp_theta1;
    p_temp(5)=  p_temp(5) + dp_theta2;
    p_temp(6) =  p_temp(6) + dp_theta2;
    p_temp(7) = p_temp(7) - dp_theta2;
    p_temp(8) = p_temp(8) - dp_theta2;
    p_temp(9)=  p_temp(9) + dp_theta3;
    p_temp(10) =  p_temp(10) + dp_theta3;
    p_temp(11) = p_temp(11) - dp_theta3;
    p_temp(12) = p_temp(12) - dp_theta3;
    p_temp(13) = p_temp(13) + dp_theta4;
    p_temp(14) = p_temp(14) - dp_theta4;
    

     dp_dis = ddisdest / k_dpddis;   %no need to adjust dis by theta because sum(p_temp) = 0
    dp_diss(1)=dp_dis / 20;
    dp_diss(2)=dp_dis / 20;
    dp_diss(3)=dp_dis / 20;
    dp_diss(4)=dp_dis / 20;
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
     max_p=min([p_now(13),p_now(14)]);
     min_p=3500-max([p_now(13),p_now(14)]);
     dp_diss(4)=max(dp_diss(4),-max_p);
     dp_diss(4)=min(dp_diss(4),min_p);
     p_temp(13)=p_temp(13)+dp_diss(4);
     p_temp(14)=p_temp(14)+dp_diss(4);
     p_max=2500;
     for i=1:14
         max_p=p_now(i);
         if i>8
             p_max=3500;
         end
         
         min_p=p_max-p_now(i);
         p_temp(i)=max(p_temp(i),-max_p);
         p_temp(i)=min(p_temp(i),min_p);
     end
     
         
     
     
     
     
    %k_dpdy  tai da le 
%     difinx =(4*(p_temp(2)+p_temp(1)-p_temp(4)-p_temp(3))+ 5*(p_temp(6)+p_temp(5)-p_temp(8)-p_temp(7)) + 7*(p_temp(10)+p_temp(9)-p_temp(12)-p_temp(11)) + 18*(p_temp(13)-p_temp(14)) ) * k_dpdy;       %注意现在的正负的意义变了
    difinx =(4*(p_temp(2)+p_temp(1)-p_temp(4)-p_temp(3))+ 5*(p_temp(6)+p_temp(5)-p_temp(8)-p_temp(7)) + 7*(p_temp(10)+p_temp(9)-p_temp(12)-p_temp(11)) + 18*(p_temp(13)-p_temp(14)) ) * k_dpdy; 
    dp_y = ( dxdest - difinx ) / k_dpdy;
    p_temp(1) =  p_temp(1) + dp_y / 100;
    p_temp(2)=  p_temp(2) + dp_y / 100;
    p_temp(3)=  p_temp(3) - dp_y/ 100;
    p_temp(4)=  p_temp(4) - dp_y / 100;
    p_temp(5)=  p_temp(5) + dp_y / 100;
    p_temp(6) =  p_temp(6) + dp_y / 100;
    p_temp(7) = p_temp(7) - dp_y/ 100;
    p_temp(8) = p_temp(8) - dp_y / 100;
    p_temp(9)=  p_temp(9) + dp_y / 100;
    p_temp(10) =  p_temp(10) + dp_y / 100;
    p_temp(11) = p_temp(11) - dp_y / 100;
    p_temp(12) = p_temp(12) - dp_y / 100;
    %3
    p_temp(13) = p_temp(13) + 3*dp_y / 100;
    p_temp(14) = p_temp(14) - 3*dp_y / 100;
    p_temp(15)=0;
    p_temp(16)=0;
    
    
    
    