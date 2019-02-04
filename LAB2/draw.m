function q = draw(x,y,d,fig,oldv)
    figure(fig);
    delete(oldv);
    l = 100;
    vPx = l*cos(d);
    vPy = l*sin(d);
    hold on;
    plot(x,y,'bo');
    q=quiver (x,y, vPx, vPy, 0, 'r');
    hold off;
end 