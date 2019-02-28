clc; clear; close all;
%make websocket connection to robot
client = WSClient('ws://192.168.4.1:81/');
% connect the client to server(robot)
client.connect()
minx = 1000;
maxx = -1000;
miny = 1000;
maxy = -1000;

client.send('#L');
tic;
while true
    % send command to robot for infomation
    client.send('@');
    % Get data from robot
    ntry = 0;
    while true
        ntry = ntry+1;
        
        pause(0.01)
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
    data = str2num(info)
    if (minx > data(5)) 
        minx = data(5);
    end
    if (maxx < data(5)) 
        maxx = data(5);
    end
    if (miny > data(6)) 
        miny = data(6);
    end
    if (maxy < data(6)) 
        maxy = data(6);
    end
    if (toc > 5) 
        client.send('#S');
        client.send('#S');
        client.send('#S');
        client.send('#S');
        client.send('#S');
        client.send('#S');
        client.send('#S');
        client.close();
        break;
    end
end
xshift = (minx + maxx)/2
yshift = (miny + maxy)/2
