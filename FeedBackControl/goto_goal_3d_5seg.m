function [p_now,k_p]=goto_goal_3d_5seg(goal,time,p_now,k_p)

    x_dest = goal(1);
    y_dest = goal(2);
    z_dest = goal(3);
    ph_dest = goal(4);%ph in xz
    theta_dest = goal(5); %theta in yz
    state_list=[];
   
    k_dpddis = k_p(1);
    k_dpdph = k_p(2) ;
    k_dpdtheta= k_p(3);  
    k_dpdx = k_p(4) ;
    k_dpdy = k_p(5) ;

    k_dpddis_old=k_dpddis;
    k_dpdph_old=k_dpdph;
    k_dpdtheta_old=k_dpdtheta;
    k_dpdx_old=k_dpdx;
    k_dpdy_old=k_dpdy;
    for i = 1:time
        k_dpddis = k_p(1);
        k_dpdph = k_p(2) ;
        k_dpdtheta= k_p(3);  
        k_dpdx = k_p(4) ;
        k_dpdy = k_p(5) ;
    
        %Get a picture
        if i==1
            [body1, body2] = mcs_date_2_body();
            %zhu yi IP
            [stateone] = deal_data_from_mcs_2body( body1,body2)
            state_list=[state_list;stateone];
        end
        X_error = x_dest-stateone(1)  ;
        Y_error =  y_dest-stateone(2) ;
        Z_error= z_dest-stateone(3);
        ph_error = ph_dest-stateone(4)  ;
        theta_error=theta_dest-stateone(5);

        %Record the current Coordinate of the arm's tail
        ptail_now = stateone;

        %Use the current X and Y to set the coordinate system for describing the change tendency of the arm
        [UXvectorDis,UvectorX] = SetCoordinateS_simple(ptail_now(1),ptail_now(3));
        [UYvectorDis,UvectorY] = SetCoordinateS_simple(ptail_now(2),ptail_now(3));

        ptail_old=ptail_now;
        p_old = p_now;

        %Virtual iteration
        %Give virtual parameters values to start iteration
        ptail_virtual=ptail_now;

        for j = 1:5
            %Set coordinate system to calculate disdest and dxdest
            [vxdis,vx] = SetCoordinateS_simple(ptail_virtual(1),ptail_virtual(3));
            [vydis,vy] = SetCoordinateS_simple(ptail_virtual(2),ptail_virtual(3));
            ptail_xz = [ptail_virtual(1);ptail_virtual(3)];
            ptail_yz = [ptail_virtual(2);ptail_virtual(3)];
            
            vxz_dest = [x_dest;z_dest] - ptail_xz;
            vyz_dest = [y_dest;z_dest]- ptail_yz ;
            
            xz_dxdest=vx'*vxz_dest;
            xz_ddisdest=vxdis'*vxz_dest*0.5;
            xz_dphdest=ph_dest-ptail_virtual(4);
            
            yz_dydest=vy'*vyz_dest;
            yz_ddisdest=vydis'*vyz_dest*0.5;
            yz_dthetadest=theta_dest-ptail_virtual(5);
            
            p_change_xz=AdjustPressure_for5seg_XZ(xz_dphdest, xz_ddisdest, xz_dxdest, k_dpdph, k_dpddis, k_dpdx, p_now);
            p_change_yz=AdjustPressure_for5seg_YZ(yz_dthetadest, yz_ddisdest, yz_dydest, k_dpdtheta, k_dpddis, k_dpdy, p_now);
            

            %Use the change of p to calculate change of x, ph, and dis
            p_change=p_change_xz+p_change_yz;
            
            xz_dis=sum(p_change)*k_dpddis*0.5;
            xz_x=(4*(p_change(2)+p_change(4)-p_change(1)-p_change(3))+ 5*(p_change(6)+p_change(8)-p_change(5)-p_change(7)) + 7*(p_change(10)+p_change(12)-p_change(9)-p_change(11)) ) * k_dpdx; 
            xz_ph=(2*(p_change(2)+p_change(4)-p_change(1)-p_change(3))+ (p_change(6)+p_change(8)-p_change(5)-p_change(7)) + (p_change(10)+p_change(12)-p_change(9)-p_change(11)) )*k_dpdph;
            
            yz_dis=sum(p_change)*k_dpddis*0.5;
            yz_y=(4*(p_change(2)+p_change(1)-p_change(4)-p_change(3))+ 5*(p_change(6)+p_change(5)-p_change(8)-p_change(7)) + 7*(p_change(10)+p_change(9)-p_change(12)-p_change(11)) + 18*(p_change(13)-p_change(14)) )*k_dpdy;
            yz_theta=(2*(p_change(2)+p_change(1)-p_change(4)-p_change(3))+ 1*(p_change(6)+p_change(5)-p_change(8)-p_change(7)) + 1*(p_change(10)+p_change(9)-p_change(12)-p_change(11)) + 2*(p_change(13)-p_change(14)) )*k_dpdtheta;
            %写的累赘了
            %dis方向应该换一种处理方式
            ptail_virtual(1)=ptail_virtual(1)+vxdis(1)*xz_dis*2+vx(1)*xz_x;
            ptail_virtual(3)=ptail_virtual(3)+vxdis(2)*xz_dis+vx(2)*xz_x;
            ptail_virtual(4)=ptail_virtual(4)+xz_ph;
            ptail_virtual(2)=ptail_virtual(2)+vydis(1)*yz_dis*2+vy(1)*yz_y;
            ptail_virtual(3)=ptail_virtual(3)+vydis(2)*yz_dis+vy(2)*yz_y;
            ptail_virtual(5)=ptail_virtual(5)+yz_theta;
                  
            p_now=p_now+p_change;
            
        end
        
        p_now=fix(p_now);
        
        for n = 1: 16
           if(p_now(n) <0)
               p_now(n) = 0;
           end
        end
        for n = 1: 8
           if(p_now(n) >2500)
               p_now(n) = 2500;
           end
        end
        for n = 9: 16
           if(p_now(n) >3500)
               p_now(n) = 3500;
           end
        end
        p_now(16)=0;

        p_now
        pressure_for_16_05s(p_now);
        
        
        [body1, body2] = mcs_date_2_body();
        
        [stateone] = deal_data_from_mcs_2body( body1,body2)
        state_list=[state_list;stateone];

        %Record the current Coordinate of the arm's tail
        ptail_now = stateone;
        
        %Calculate the real change of x, dis and ph
        tail_change = ptail_now - ptail_old;
        x_change = [tail_change(1) tail_change(3)] * UvectorX; 
        y_change=[tail_change(2) tail_change(3)]*UvectorY;
        dis_change=[tail_change(1) tail_change(3)]*UXvectorDis*0.5+[tail_change(2) tail_change(3)]*UYvectorDis*0.5;
        ph_change=tail_change(4);
        theta_change=tail_change(5);
        p_change = p_now - p_old;

        %Calculate the kValues according to the real changes above
        k_dpddis = ( dis_change ) / ( 2*(p_change(2)+p_change(1)+p_change(4)+p_change(3))+ 1*(p_change(6)+p_change(5)+p_change(8)+p_change(7)) + 1*(p_change(10)+p_change(9)+p_change(12)+p_change(11)) + 2*(p_change(13)+p_change(14)) + 0.1 );    %Add 0.1 to avoid unavailable dividend

        k_dpdph = ( ph_change ) / (2*(p_change(2)+p_change(4)-p_change(1)-p_change(3))+ (p_change(6)+p_change(8)-p_change(5)-p_change(7)) + (p_change(10)+p_change(12)-p_change(9)-p_change(11)) + 0.1 ); %Add 0.1 to avoid unavailable dividend
        k_dpdtheta = (theta_change ) / (2*(p_change(2)+p_change(1)-p_change(4)-p_change(3))+ 1*(p_change(6)+p_change(5)-p_change(8)-p_change(7)) + 1*(p_change(10)+p_change(9)-p_change(12)-p_change(11)) + 2*(p_change(13)-p_change(14))+ 0.1  );

        k_dpdx = ( x_change) / (4*(p_change(2)+p_change(4)-p_change(1)-p_change(3))+ 5*(p_change(6)+p_change(8)-p_change(5)-p_change(7)) + 7*(p_change(10)+p_change(12)-p_change(9)-p_change(11)) + 0.1 ); %Add 0.1 to avoid unavailable dividend
        k_dpdy = ( y_change ) / (4*(p_change(2)+p_change(1)-p_change(4)-p_change(3))+ 5*(p_change(6)+p_change(5)-p_change(8)-p_change(7)) + 7*(p_change(10)+p_change(9)-p_change(12)-p_change(11)) + 18*(p_change(13)-p_change(14)) + 0.1 );
        
        scale=max(k_dpddis_old/k_dpddis,k_dpddis/k_dpddis_old);
        if scale>0&&scale<2
            k_dpddis=k_dpddis/1.2;
            k_dpddis=max( k_dpddis, 0.01 );
        else 
            k_dpddis = max( k_dpddis, 0.02 );
        end
        
        scale=max(k_dpdph_old/k_dpdph,k_dpdph/k_dpdph_old);
        if scale>0&&scale<2
            k_dpdph=k_dpdph/1.2;
            k_dpdph=max( k_dpdph, 0.0002 );
        else 
            k_dpdph = max( k_dpdph, 0.0004 );
        end
        
        scale=max(k_dpdtheta_old/k_dpdtheta,k_dpdtheta/k_dpdtheta_old);
        if scale>0&&scale<2
            k_dpdtheta=k_dpdtheta/1.2;
            k_dpdtheta=max( k_dpdtheta, 0.0002 );
        else 
            k_dpdtheta = max( k_dpdtheta, 0.0004 );
        end
        
        scale=max(k_dpdx_old/k_dpdx,k_dpdx/k_dpdx_old);
        if scale>0&&scale<2
            k_dpdx=k_dpdx/1.2;
            k_dpdx=max( k_dpdx, 0.01 );
        else 
            k_dpdx = max( k_dpdx, 0.02 );
        end
        
        scale=max(k_dpdy_old/k_dpdy,k_dpdy/k_dpdy_old);
        if scale>0.5&&scale<2
            k_dpdy=k_dpdy/1.5;
            k_dpdy=max( k_dpdy, 0.003 );
        else 
            k_dpdy = max( k_dpdy, 0.01 );
        end
        
        k_dpddis_old=k_dpddis;
        k_dpdph_old=k_dpdph;
        k_dpdtheta_old=k_dpdtheta;
        k_dpdx_old=k_dpdx;
        k_dpdy_old=k_dpdy;

        %If the kValues are too small then throw them
        k_p(1) = max( k_dpddis, 0.01 );
        k_p(2) = max( k_dpdph, 0.0002 );
        k_p(3)=max(k_dpdtheta,0.0002);
        k_p(4) = max( k_dpdx, 0.01 );
        k_p(5) = max( k_dpdy, 0.01 );  
    end

end

