function out = app_sample(reference,start,goal,c,keepDense)
%APP_SAMPLE Pure pursuit (A4), steering slew bound (5), bicycle model (2).
if nargin<5, keepDense=false; end
out=struct('path',zeros(0,4),'carrots',zeros(0,2),'dense',zeros(0,4),'valid',false);
if size(reference,1)<2, return; end
reference(end+1,:)=goal(1:2)+c.extension*[cos(goal(3)) sin(goal(3))];
keep=[true; hypot(diff(reference(:,1)),diff(reference(:,2)))>1e-10];
reference=reference(keep,:); if size(reference,1)<2, return; end
s=[0;cumsum(hypot(diff(reference(:,1)),diff(reference(:,2))))];
n=max(2,ceil(s(end)/c.reference_spacing)+1); ref=zeros(n,2); j=1;
for i=1:n
    d=s(end)*(i-1)/(n-1);
    while j+1<numel(s) && s(j+1)<d, j=j+1; end
    f=(d-s(j))/(s(j+1)-s(j)); ref(i,:)=reference(j,:)+f*(reference(j+1,:)-reference(j,:));
end
cur=start; latest=1; dt=c.simulation_dt/c.integration_steps; exhausted=false;
path=zeros(c.max_tracking_steps,4); carrots=zeros(c.max_tracking_steps,2); count=0;
if keepDense, dense=zeros(1+c.max_tracking_steps*c.integration_steps,4); dense(1,:)=cur; end
for step=1:c.max_tracking_steps
    distances=hypot(ref(latest:end,1)-cur(1),ref(latest:end,2)-cur(2));
    [~,nearest]=min(distances); nearest=nearest+latest-1;
    latest=min(nearest+1,size(ref,1)); carrot=nearest;
    while carrot<=size(ref,1) && hypot(ref(carrot,1)-cur(1),ref(carrot,2)-cur(2))<c.lookahead
        carrot=carrot+1;
    end
    if carrot>size(ref,1), exhausted=true; break; end
    cp=ref(carrot,:); alpha=atan2(cp(2)-cur(2),cp(1)-cur(1))-cur(3);
    desired=max(-c.max_steer,min(c.max_steer,atan2(2*c.wheelbase*sin(alpha),c.lookahead)));
    for j=1:c.integration_steps
        next=cur(4)+max(-c.max_steer_rate*dt,min(c.max_steer_rate*dt,desired-cur(4)));
        midphi=cur(4)+max(-c.max_steer_rate*dt/2,min(c.max_steer_rate*dt/2,desired-cur(4)));
        yaw=c.speed/c.wheelbase*tan(midphi)*dt;
        cur(1)=cur(1)+c.speed*cos(cur(3)+yaw/2)*dt;
        cur(2)=cur(2)+c.speed*sin(cur(3)+yaw/2)*dt;
        cur(3)=cur(3)+yaw; cur(4)=next;
        if keepDense, dense(1+(step-1)*c.integration_steps+j,:)=cur; end
    end
    count=count+1; path(count,:)=cur; carrots(count,:)=cp;
end
out.path=path(1:count,:); out.carrots=carrots(1:count,:);
if keepDense, out.dense=dense(1:1+count*c.integration_steps,:); end
if ~exhausted||count==0, return; end
[~,last]=min(hypot(path(1:count,1)-goal(1),path(1:count,2)-goal(2)));
out.path=path(1:last,:); out.carrots=carrots(1:last,:);
if keepDense, out.dense=dense(1:1+last*c.integration_steps,:); end
out.valid=last>=2;
end
