function r = app_plan(scene,c)
%APP_PLAN Algorithms 1-3 of Li et al., IEEE TIV, 2023.
r=struct('success',false,'status','uninitialized','initial_route',zeros(0,2), ...
    'sample',struct('path',zeros(0,4),'carrots',zeros(0,2),'dense',zeros(0,4),'valid',false), ...
    'history',zeros(0,3));
if app_geometry('collision',scene.start,scene,c)||app_geometry('collision',scene.goal,scene,c)
    r.status='invalid_endpoint'; return;
end
r.initial_route=app_astar(scene,c);
if size(r.initial_route,1)<2, r.status='no_astar_route'; return; end
carrots=r.initial_route; left=c.buffer_left; right=c.buffer_right;
for iter=1:c.outer_iterations
    r.sample=app_sample(carrots,scene.start,scene.goal,c,true);
    if ~r.sample.valid, r.status='tracking_failed'; return; end
    bad=identify(r.sample.path,scene,c); segments=app_segments(r.sample.path,bad,left,right);
    r.history(end+1,:)=[iter,sum(bad),size(segments,1)];
    if isempty(segments)
        for i=1:size(r.sample.dense,1)
            if app_geometry('collision',r.sample.dense(i,:),scene,c)
                r.status='dense_collision'; return;
            end
        end
        r.success=true; r.status='success'; return;
    end
    carrots=r.sample.carrots;
    for i=1:size(segments,1)
        a=segments(i,1); b=segments(i,2);
        carrots(a:b,:)=polish(scene,c,r.sample,segments(i,:),bad);
    end
    left=left+c.buffer_increment; right=right+c.buffer_increment;
end
r.status='iteration_limit';
end

function bad=identify(path,scene,c)
bad=false(size(path,1),1);
for i=1:size(path,1), bad(i)=app_geometry('collision',path(i,:),scene,c); end
end

function carrots=polish(scene,c,globalSample,seg,flags)
a=seg(1); b=seg(2); n=b-a+1;
path=globalSample.path(a:b,:); carrots=globalSample.carrots(a:b,:); bad=flags(a:b);
start=path(1,:); goal=path(end,:);
for iter=1:c.inner_iterations
    for i=1:n
        if ~bad(i), continue; end
        rate=app_geometry('rates',path(i,:),scene,c);
        id=app_matched_carrot(carrots,i,abs(rate(1)-rate(2))*c.traceback_length);
        sign=1; if rate(1)<rate(2), sign=-1; end
        carrots(id,1)=carrots(id,1)+sign*c.nudge*sin(path(i,3));
        carrots(id,2)=carrots(id,2)-sign*c.nudge*cos(path(i,3));
    end
    sample=app_sample(carrots,start,goal,c);
    if ~sample.valid, break; end
    % Paired resampling keeps the smooth-point / tracked-carrot association.
    ids=floor((0:n-1)'*(size(sample.path,1)-1)/(n-1)+0.5)+1;
    path=sample.path(ids,:); carrots=sample.carrots(ids,:);
    bad=identify(path,scene,c); if ~any(bad), break; end
end
end
