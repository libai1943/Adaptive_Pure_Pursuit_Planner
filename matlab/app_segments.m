function segments = app_segments(path,bad,left,right)
%APP_SEGMENTS Grow and merge seeds by accumulated arc length, (6)-(7).
n=size(path,1); mask=false(n,1);
for i=find(bad(:))'
    a=i; b=i; s=0;
    while a>1 && s<left, s=s+hypot(path(a,1)-path(a-1,1),path(a,2)-path(a-1,2)); a=a-1; end
    s=0;
    while b<n && s<right, s=s+hypot(path(b,1)-path(b+1,1),path(b,2)-path(b+1,2)); b=b+1; end
    mask(a:b)=true;
end
edges=diff([false;mask;false]); segments=[find(edges==1),find(edges==-1)-1];
end
