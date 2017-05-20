function [body1, body2, body3, body4, body5, body6] = GetBodyFromMCS( )

	global theClient;

    java.lang.Thread.sleep(20);
    for i = 1 : 20
        java.lang.Thread.sleep(2);
        frameOfData = theClient.GetLastFrameOfData();
        for j = 1 : 6
            rigidBodyData = frameOfData.RigidBodies(j);
            body(i,j,1,1) = rigidBodyData.Markers(1).x;
            body(i,j,1,2) = rigidBodyData.Markers(1).y;
            body(i,j,1,3) = rigidBodyData.Markers(1).z;
            body(i,j,2,1) = rigidBodyData.Markers(2).x;
            body(i,j,2,2) = rigidBodyData.Markers(2).y;
            body(i,j,2,3) = rigidBodyData.Markers(2).z;
            body(i,j,3,1) = rigidBodyData.Markers(3).x;
            body(i,j,3,2) = rigidBodyData.Markers(3).y;
            body(i,j,3,3) = rigidBodyData.Markers(3).z;
        end
    end

    temp = ones(1, 20);
    bodyR = zeros(6, 3, 3);
    for i = 1 : 6
        for j = 1 : 20
            temp(j) = body(j,i,1,1);
        end
        bodyR(i,1,1) = median(temp,2);
        for j = 1 : 20
            temp(j) = body(j,i,1,2);
        end
        bodyR(i,1,2) = median(temp,2);
        for j = 1 : 20
            temp(j) = body(j,i,1,3);
        end
        bodyR(i,1,3) = median(temp,2);
        for j = 1 : 20
            temp(j) = body(j,i,2,1);
        end
        bodyR(i,2,1) = median(temp,2);
        for j = 1 : 20
            temp(j) = body(j,i,2,2);
        end
        bodyR(i,2,2) = median(temp,2);
        for j = 1 : 20
            temp(j) = body(j,i,2,3);
        end
        bodyR(i,2,3) = median(temp,2);
        for j = 1 : 20
            temp(j) = body(j,i,3,1);
        end
        bodyR(i,3,1) = median(temp,2);
        for j = 1 : 20
            temp(j) = body(j,i,3,2);
        end
        bodyR(i,3,2) = median(temp,2);
        for j = 1 : 20
            temp(j) = body(j,i,3,3);
        end
        bodyR(i,3,3) = median(temp,2);
    end
                    
    body1 = reshape(bodyR(1,:,:), 3, 3);
    body2 = reshape(bodyR(2,:,:), 3, 3);
    body3 = reshape(bodyR(3,:,:), 3, 3);
    body4 = reshape(bodyR(4,:,:), 3, 3);
    body5 = reshape(bodyR(5,:,:), 3, 3);
    body6 = reshape(bodyR(6,:,:), 3, 3);

end