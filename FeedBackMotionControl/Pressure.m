function [] = Pressure( pNow )
	%pNow 是一个1*20的行向量，各元素对应气压关系和图注一致
	%在这个函数内需要将这个行向量变换成实际充气所使用的向量
	%并且调用pressure来完成充气
    
    %Fixit:SEGNUM should be used here
    global MAXP;
    
    pR = zeros( 1, 16 );
    
    pR(1) = ( pNow(1) + pNow(5) ) / 2;
    pR(2) = ( pNow(2) + pNow(6) ) / 2;
    pR(3) = ( pNow(3) + pNow(7) ) / 2;
    pR(4) = ( pNow(4) + pNow(8) ) / 2;
    
    for i = 5 : 12
        pR(i) = pNow(i+4);
    end
    
    pR(13) = ( pNow(17) + pNow(18) ) / 2;
    pR(14) = ( pNow(19) + pNow(20) ) / 2;
    
    pR = fix( pR );
    
    for i = 1:16
        if( pR(i) > MAXP )
            pR(i) = MAXP;
        end
        if( pR(i) < 0 )
            pR(i) = 0;
        end
    end
    
    pressure_for_16_05s( pR );

    pNow(1) = ( pNow(1) + pNow(5) ) / 2;
    pNow(2) = ( pNow(2) + pNow(6) ) / 2;
    pNow(3) = ( pNow(3) + pNow(7) ) / 2;
    pNow(4) = ( pNow(4) + pNow(8) ) / 2;
    pNow(5) = pNow(1);
    pNow(6) = pNow(2);
    pNow(7) = pNow(3);
    pNow(8) = pNow(4);

    pNow(17) = ( pNow(17) + pNow(18) ) / 2;
    pNow(18) = pNow(17);
    pNow(19) = ( pNow(19) + pNow(20) ) / 2;
    pNow(20) = pNow(19);
end