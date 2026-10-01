function best = app_matched_carrot(carrots,i,tb)
%APP_MATCHED_CARROT Equation (9), along the carrot path, nearest arc distance.
best=i; s=0; error=abs(tb);
for j=i-1:-1:1
    s=s+hypot(carrots(j,1)-carrots(j+1,1),carrots(j,2)-carrots(j+1,2));
    e=abs(tb-s);
    if e<error-1e-10, error=e; best=j; end
end
end
