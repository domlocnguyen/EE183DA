function h = mag2heading(mx,my)
  h = atan2(mx,my);
  if(h < 0)
     h = h + 2*pi;
  end
  
  if(h > 2*pi)
    h = h - 2*pi;
  end
end