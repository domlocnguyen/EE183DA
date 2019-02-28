function [H,C] = pos2sensorMatrix(x,Lx,Ly)
    % Convert from pos to sensor signal
    % Current position, x,y and heading angle
    rx = x(1);
    ry = x(2);
    th = x(3);
    % Compute Threshold Angle
    th_abs = abs(x(3));
    tht = atan((Ly-ry) / (Lx - rx));
    thb = atan((ry) / (Lx - rx));
    thr = atan((Lx-rx)/ry);
    thl = atan(rx/ry);
    c = cos(th_abs);
    s = sin(th_abs);
    if (th >= 0 && th_abs <= tht && th_abs <= thr)
        disp("Case1: th >= 0 && th_abs <= tht && th_abs <= thr");      
        y = [(Lx-rx)/c; ry/c; th];
    elseif (th >= 0 && th_abs <= tht && th_abs >= thr)
        disp("Case2: th >= 0 && th_abs <= tht && th_abs >= thr");    
        y = [(Lx-rx)/c; (Lx-rx)/s; th];
    elseif (th >= 0 && th_abs >= tht && th_abs <= thr)
        disp("Case3: th >= 0 && th_abs >= tht && th_abs <= thr");    
        y = [(Lx-ry)/s; ry/c; th];
    elseif (th >= 0 && th_abs >= tht && th_abs >= thr)
        disp("Case4: th >= 0 && th_abs >= tht && th_abs >= thr");    
        y = [(Lx-ry)/s; (Lx-rx)/s; th];
    elseif (th <= 0 && th_abs <= thb && th_abs <= thl)
        disp("Case5: th <= 0 && th_abs <= thb && th_abs <= thl");    
        y = [(Lx-rx)/c; ry/c; th];
    elseif (th <= 0 && th_abs <= thb && th_abs >= thl)
        disp("Case6: th <= 0 && th_abs <= thb && th_abs >= thl");    
        y = [(Lx-rx)/c; rx/s; th];
    elseif (th <= 0 && th_abs >= thb && th_abs <= thl)
        disp("Case7: th <= 0 && th_abs >= thb && th_abs <= thl");    
        y = [ry/s; ry/c; th];
    elseif (th <= 0 && th_abs >= thb && th_abs >= thl)
        disp("Case8: th <= 0 && th_abs >= thb && th_abs >= thl");    
        y = [ry/s; rx/s; th];
    end
    
    % Calculate Matrix for linearlization (around x)
    x0 = x;
    y0 = y;
    % unpack
    rx = x(1);
    ry = x(2);
    th = x(3);
    th_abs = abs(th);
    rx0 = x0(1);
    ry0 = x0(2);
    th0 = x0(3);
    lx0 = y0(1);
    ly0 = y0(2);
    
    c = cos(th0);
    s = sin(th0);
    th0_abs = abs(th0);
    s_abs = sin(th0_abs);
    % Get threshold angle
    tht = atan((Ly-ry) / (Lx - rx));
    thb = atan((ry) / (Lx - rx));
    thr = atan((Lx-rx)/ry);
    thl = atan(rx/ry);
    if (th >= 0 && th_abs <= tht && th_abs <= thr)
        disp("Case1: th >= 0 && th_abs <= tht && th_abs <= thr");
        a = (Lx-rx0)*s/c^2;
        b = ry0*s/c^2;
        H = [-1/c, 0, a;
           0, 1/c, b;
           0, 0, 1
           ];
        C = [rx0/c - a*th0 + lx0;
           -ry0/c - b*th0 + ly0;
           0];
    elseif (th >= 0 && th_abs <= tht && th_abs >= thr)
        disp("Case2: th >= 0 && th_abs <= tht && th_abs >= thr");
        a = (Lx-rx0)*s/c^2;
        b = (Lx-rx0)*c/s_abs^2;
        H = [-1/c, 0, a;
            -1/s_abs, 0, -(th0/th0_abs)*b;
            0, 0, 1];
        C = [rx0/c - a*th0 + lx0;
            rx0/s_abs + b*th0_abs + ly0;
            0];
    elseif (th >= 0 && th_abs >= tht && th_abs <= thr)  
        disp("Case3: th >= 0 && th_abs >= tht && th_abs <= thr");
        a = (Ly-ry0)*c/s_abs^2;
        b = ry0*s/c^2;
        H = [0, -1/s_abs, -(th0/th0_abs)*a;
            0, 1/c, b;
            0, 0, 1];
        C = [ry0/s_abs + a*th0_abs + lx0;
            -ry0/c - b*th0 + ly0;
            0];
    elseif (th >= 0 && th_abs >= tht && th_abs >= thr)
        disp("Case4: th >= 0 && th_abs >= tht && th_abs >= thr");
        a = (Ly-ry0)*c/s_abs^2;
        b = (Lx-rx0)*c/s_abs^2;
        H = [0, -1/s_abs, (-th0/th0_abs)*a;
            -1/s_abs, 0, (-th0/th0_abs)*b
            0, 0, 1
            ];
        C = [ry0/s_abs + a*th0_abs + lx0;
            rx0/s_abs + b*th0_abs + ly0;
            0];
    elseif (th <= 0 && th_abs <= thb && th_abs <= thl)
        disp("Case5: th <= 0 && th_abs <= thb && th_abs <= thl");
        a = (Lx-rx0)*s/c^2;
        b = ry0*s/c^2;
        H = [-1/c, 0, a;
           0, 1/c, b;
           0, 0, 1
           ];
        C = [rx0/c - a*th0 + lx0;
           -ry0/c - b*th0 + ly0;
           0];
    elseif (th <= 0 && th_abs <= thb && th_abs >= thl)
        disp("Case6: th <= 0 && th_abs <= thb && th_abs >= thl");
        a = (Lx-rx0)*s/c^2;
        b = rx0*c/s_abs^2;
        H = [-1/c, 0, a;
            1/s_abs, 0, (-th/th_abs)*b;
            0, 0, 1];
        C = [rx0/c - a*th0 + lx0;
            -rx0/s_abs + b*th0_abs + ly0;
            0];
    elseif (th <= 0 && th_abs >= thb && th_abs <= thl)
        disp("Case7: th <= 0 && th_abs >= thb && th_abs <= thl");
        a = ry0*c/s_abs^2;
        b = ry0*s/c^2;
        H = [0, 1/s_abs, (-th0/th0_abs)*a;
            0, 1/c, b;
            0, 0, 1];
        C = [-ry0/s_abs + a*th0_abs + lx0;
            -ry0/c - b*th0 + ly0;
            0];
    elseif (th <= 0 && th_abs >= thb && th_abs >= thl)
        disp("Case8: th <= 0 && th_abs >= thb && th_abs >= thl");
        a = ry0*c/s_abs^2;
        b = rx0*c/s_abs^2;
        H = [0, 1/s_abs, (-th0/th0_abs)*a;
            1/s_abs, 0 (-th0/th0_abs)*b;
            0, 0, 1];
        C = [-ry0/s_abs + a*th0_abs + lx0;
            -rx0/s_abs + b*th0_abs + ly0;
            0];
    end
    
end