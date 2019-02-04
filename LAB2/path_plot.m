clc; clear; close all;
%make websocket connection to robot
client = WSClient('ws://192.168.4.1:81/');
% connect the client to server(robot)
client.connect()
% Prepare plot to draw position of robot
map = figure('Name', 'State Estimation');
axes('Position',[0.1 0.1 0.85 0.85],'Box','on');
axis([0 750 0 750]);

%Data path 0: F, 1: L, 2:R, 3:B, 4:S
D = [0, 2;
    1,0.6;
    0,1;
    2,0.6;
    3,1.8;
    4,2;
    5,1000];

%define constants
L=90; %distance between left and right
dt = 0.1;
maxspeed = 182; % mm/s
lastdraw = [];
lastdraw = draw(x(1),x(2),x(3),map,lastdraw);

Lx = 597; %size of box
Ly = 457; %size of box
x=[100 100 0]'; %initial position
lastdraw = draw(x(1),x(2),x(3),map,lastdraw); % Plot initial position
y=[497 65 0]'; %sensor at initial position
u=[0 0]'; %input velocity (mm/s)
tht = atan((Ly-x(2))/(Lx-x(1))); % top angle of current position
thb = atan(x(2)/(Lx-x(1))); % bottom angle of current position
thr = atan((Lx-x(1))/x(2)); % right angle of current position
thl = atan(x(1)/x(2)); % right angle of current position

% calib the intial angle, and get the first sensor signal
ntry = 0;
while true
    ntry = ntry+1;
    % send command to robot for infomation
    client.send('@');
    % waiting dt ms to make sure the request info ready
    pause(dt);
    info = client.Message;
    if sum(size(info))~=0
        break
    end
    if ntry > 10
        client.close();
        break
    end
end

data = str2num(info);
angleShift = data(5);
y=[data(4), data(3), 0];

sD = size(D);
dt=0.1;
n=0; %counting time
curC = 1;
stop = 0;

% Update state
for z=0:30000  %simulation time = dt*30000
    % Control the robot go by data path in D
    addT = 0;
    for j = 1: sD(1)
        addT = addT + D(j,2);
        if (addT - D(j,2) <= n && addT >= n )
            switch (D(j,1))
                case 0
                    client.send('#F')
                case 1
                    client.send('#L')
                case 2
                    client.send('#R')
                case 3
                    client.send('#B')
                case 4
                    client.send('#S')
                otherwise
                    client.send('#S')
                    stop = 1;
            end
            break;
        end
    end
    n=n+dt;
    
    % Get data from robot
    ntry = 0;
    while true
        ntry = ntry+1;
        % send command to robot for infomation
        client.send('@');
        % waiting dt ms to make sure the request info ready
        pause(dt);
        info = client.Message;
        if sum(size(info))~=0
            break
        end
        if ntry > 100
            client.close();
            break
        end
    end
    % Process data received
    data = str2num(info);
    PWML = data(1);
    PWMR = data(2);
    %convert from PWM Left to mm/s
    if (PWML >115)
        vl = maxspeed;
    else
        if (PWML > 65)
            vl = 4.892*PWML - 440.28;
        else
        vl=-maxspeed;
        end
    end
    %convert from PWM Left to mm/s
    if (PWMR >115)
        vr = maxspeed;
    else
        if (PWMR > 65)
            vr = 4.892*PWMR - 440.28;
        else
            vr=-maxspeed;
        end
    end

    u = [vl, vr]';
    y = [data(4), data(3), data(5)-angleShift]';
    B = [cos(x(3))/2 cos(x(3))/2; 
        sin(x(3))/2 sin(x(3))/2;
        -1/L 1/L];

    Qb = [0.32882 0;
    0 0.306615];
 
    x = eye(3)*x + dt*B*u;
    lastdraw = draw(x(1),x(2),x(3),map,lastdraw);
    % Stop when end of path
    if (stop == 1)
       client.send('#S');
       client.close();
       break;
    end
end

client.close()