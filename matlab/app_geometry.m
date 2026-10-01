function value = app_geometry(action, varargin)
%APP_GEOMETRY Deterministic convex geometry shared in meaning with cpp/app.cpp.
switch action
    case 'footprint', value = footprint(varargin{:});
    case 'clip', value = clip(varargin{:});
    case 'area', value = area(varargin{:});
    case 'collision', value = collision(varargin{:});
    case 'rates', value = rates(varargin{:});
    otherwise, error('Unknown geometry operation');
end
end

function p = footprint(s,c,lower,upper)
if nargin<3, lower=-1; upper=1; end
p = [-c.rear_overhang lower*c.width/2; c.length-c.rear_overhang lower*c.width/2; ...
    c.length-c.rear_overhang upper*c.width/2; -c.rear_overhang upper*c.width/2];
co=cos(s(3)); si=sin(s(3));
p = [s(1)+p(:,1)*co-p(:,2)*si, s(2)+p(:,1)*si+p(:,2)*co];
end

function a = area(p)
if isempty(p), a=0; return; end
q=p([2:end 1],:); a=abs(sum(p(:,1).*q(:,2)-p(:,2).*q(:,1)))*0.5;
end

function p = clip(p,boundary)
for k=1:size(boundary,1)
    if isempty(p), return; end
    a=boundary(k,:); b=boundary(mod(k,size(boundary,1))+1,:);
    out=zeros(0,2); previous=p(end,:); dp=cross2(a,b,previous);
    for j=1:size(p,1)
        current=p(j,:); dc=cross2(a,b,current); ip=dp>=0; ic=dc>=0;
        if ip~=ic
            t=dp/(dp-dc); out(end+1,:)=previous+t*(current-previous); %#ok<AGROW>
        end
        if ic, out(end+1,:)=current; end %#ok<AGROW>
        previous=current; dp=dc;
    end
    p=out;
end
end

function v=cross2(a,b,p)
v=(b(1)-a(1))*(p(2)-a(2))-(b(2)-a(2))*(p(1)-a(1));
end

function ids=candidates(p,scene)
if isempty(p), ids=[]; return; end
b=scene.boxes;
ids=find(max(p(:,1))>=b(:,1)-1e-10 & min(p(:,1))<=b(:,2)+1e-10 & ...
    max(p(:,2))>=b(:,3)-1e-10 & min(p(:,2))<=b(:,4)+1e-10)';
end

function yes=overlap(a,b)
yes=true;
for p={a,b}
    q=p{1};
    for i=1:size(q,1)
        u=q(i,:); v=q(mod(i,size(q,1))+1,:); axis=[u(2)-v(2); v(1)-u(1)];
        aa=a*axis; bb=b*axis;
        if max(aa)<min(bb)-1e-10 || max(bb)<min(aa)-1e-10, yes=false; return; end
    end
end
end

function yes=collision(s,scene,c)
p=footprint(s,c); b=scene.bounds;
yes=any(p(:,1)<b(1)-1e-10 | p(:,1)>b(2)+1e-10 | p(:,2)<b(3)-1e-10 | p(:,2)>b(4)+1e-10);
if yes, return; end
for k=candidates(p,scene)
    q=reshape(scene.triangles(k,:),2,[])';
    if overlap(p,q), yes=true; return; end
end
end

function value=rates(s,scene,c)
b=scene.bounds; bounds=[b(1) b(3);b(2) b(3);b(2) b(4);b(1) b(4)];
value=zeros(1,2); limits=[0 1;-1 0];
for side=1:2
    p=footprint(s,c,limits(side,1),limits(side,2)); inside=clip(p,bounds);
    a=c.length*c.width/2-area(inside);
    for k=candidates(inside,scene)
        q=reshape(scene.triangles(k,:),2,[])'; a=a+area(clip(inside,q));
    end
    value(side)=max(0,min(1,a/(c.length*c.width/2)));
end
end
