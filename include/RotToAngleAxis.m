function [h,theta] = RotToAngleAxis(R)

    [n,m] = size(R);
    isRotation = IsRotationMatrix(R);

    if isRotation

        trace = 0;
        for i=1:n
            trace = trace + R(i,i); 
        end

        theta = acos((trace-1)/2);

        RT = R';

        if -10^-3 <= theta && theta <= 10^-3
            %null rotation so h is arbitrary
            disp("0");
            h = [1; 0; 0];

        elseif pi-10^-3 <= theta && theta <= pi+10^-3
            
            disp("pi");
            if(sqrt((R(1,1) + 1)/2)>0)
                hx = sqrt((R(1,1) + 1)/2);
                hy = sign(hx) * sign(R(1,2))*sqrt((R(2,1)+1)/2);
                hz = sign(hx) * sign(R(1,3)) * sqrt((R(3,1)+1)/2);
                
            elseif(sqrt((R(2,2) + 1)/2)>0)
                hy = sqrt((R(2,2) + 1)/2);
                hx = sign(hy) * sign(R(2,1))*sqrt((R(1,2)+1)/2);
                hz = sign(hy) * sign(R(2,3))*sqrt((R(3,2)+1)/2);
            else
                hz = sqrt((R(3,3) + 1)/2);
                hx = sign(hz) * sign(R(3,1))*sqrt((R(1,3)+1)/2);
                hy = sign(hz) * sign(R(3,2))*sqrt((R(2,3)+1)/2);
            end
            
            
            h = [hx; hy; hz];
            h = real(h/norm(h));
           
        else
            disp("tra 0 e pi");
            diff_R_RT = R - RT;
            axial_vector = vex(diff_R_RT/2);
            h = (1 / sin(theta)) * axial_vector ;
            h = real(h/norm(h));
           
            
        end
    else
        %non rotation matrix
        theta = -5;
        h = [0;0;0];
    end

end


function a = vex(S_a)
    a = [S_a(3,2) ; S_a(1,3) ;S_a(2,1)];
end

function [sign] = sign(x)
    if(x <0)
        sign = -1;
    elseif(x > 0)
        sign = 1;
    else 
        sign = 0;
    end
end