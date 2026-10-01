function path = app_astar(scene,c)
%APP_ASTAR Eight-neighbor A* on a raster dilated by half the vehicle width.
% Node IDs, neighbor order, cost tolerance and heap tie rules match C++.
b=scene.bounds; d=c.grid_resolution;
nx=floor((b(2)-b(1))/d)+1; ny=floor((b(4)-b(3))/d)+1;
assert(nx>=2 && ny>=2 && nx*ny<=1e7, 'Unsupported grid size');
occupied=false(nx,ny); p=scene.triangles;
for k=1:size(p,1)
    t=reshape(p(k,:),2,[])'; box=scene.boxes(k,:);
    xx=max(0,ceil((box(1)-b(1))/d)):min(nx-1,floor((box(2)-b(1))/d));
    yy=max(0,ceil((box(3)-b(3))/d)):min(ny-1,floor((box(4)-b(3))/d));
    [ix,iy]=ndgrid(xx,yy); x=b(1)+ix*d; y=b(3)+iy*d; inside=true(size(x));
    for j=1:3
        a=t(j,:); z=t(mod(j,3)+1,:);
        inside=inside & ((z(1)-a(1))*(y-a(2))-(z(2)-a(2))*(x-a(1))>=-1e-10);
    end
    ids=iy(inside)*nx+ix(inside)+1; occupied(ids)=true;
end
radius=ceil(c.width/2/d); [dx,dy]=ndgrid(-radius:radius,-radius:radius);
blocked=conv2(double(occupied),double(dx.^2+dy.^2<=radius^2),'same')>0;
blocked(1:min(radius,nx),:)=true; blocked(max(1,nx-radius+1):end,:)=true;
blocked(:,1:min(radius,ny))=true; blocked(:,max(1,ny-radius+1):end)=true;
path=zeros(0,2);
if any(scene.start(1:2)<b([1 3])) || any(scene.start(1:2)>b([2 4])) || ...
        any(scene.goal(1:2)<b([1 3])) || any(scene.goal(1:2)>b([2 4])), return; end
startXY=floor((scene.start(1:2)-b([1 3]))/d+0.5);
goalXY=floor((scene.goal(1:2)-b([1 3]))/d+0.5);
if any(startXY>=[nx ny]) || any(goalXY>=[nx ny]), return; end
start=startXY(2)*nx+startXY(1)+1; goal=goalXY(2)*nx+goalXY(1)+1;
if blocked(start)||blocked(goal), return; end
total=nx*ny; g=inf(total,1); parent=zeros(total,1); closed=false(total,1);
heap=zeros(total,3); count=0; g(start)=0; push(start);
dx=[1 0 -1 0 1 -1 -1 1]; dy=[0 1 0 -1 1 1 -1 -1];
while count>0
    entry=pop(); u=entry(2);
    if closed(u)||entry(3)>g(u)+1e-10, continue; end
    if u==goal, break; end
    closed(u)=true; x=mod(u-1,nx); y=floor((u-1)/nx);
    for k=1:8
        xx=x+dx(k); yy=y+dy(k);
        if xx<0||xx>=nx||yy<0||yy>=ny, continue; end
        v=yy*nx+xx+1;
        if blocked(v)||closed(v), continue; end
        if k>=5 && (blocked(y*nx+xx+1)||blocked(yy*nx+x+1)), continue; end
        step=1; if k>=5, step=sqrt(2); end
        candidate=g(u)+d*step;
        if candidate<g(v)-1e-10, g(v)=candidate; parent(v)=u; push(v); end
    end
end
if ~isfinite(g(goal)), return; end
ids=goal;
while parent(ids(end))~=0, ids(end+1)=parent(ids(end)); end %#ok<AGROW>
ids=fliplr(ids)'; path=[b(1)+mod(ids-1,nx)*d,b(3)+floor((ids-1)/nx)*d];
path(1,:)=scene.start(1:2); path(end,:)=scene.goal(1:2);

    function push(id)
        point=[b(1)+mod(id-1,nx)*d,b(3)+floor((id-1)/nx)*d];
        goalPoint=b([1 3])+goalXY*d;
        h=hypot(point(1)-goalPoint(1),point(2)-goalPoint(2));
        value=[floor((g(id)+h)*1e9+0.5),id,g(id)];
        count=count+1; at=count;
        while at>1
            up=floor(at/2); if ~less(value,heap(up,:)), break; end
            heap(at,:)=heap(up,:); at=up;
        end
        heap(at,:)=value;
    end
    function value=pop()
        value=heap(1,:); last=heap(count,:); count=count-1; at=1;
        while at*2<=count
            child=at*2;
            if child<count && less(heap(child+1,:),heap(child,:)), child=child+1; end
            if ~less(heap(child,:),last), break; end
            heap(at,:)=heap(child,:); at=child;
        end
        heap(at,:)=last;
    end
end
function yes=less(a,b)
yes=a(1)<b(1)||(a(1)==b(1)&&(a(2)<b(2)||(a(2)==b(2)&&a(3)<b(3))));
end
