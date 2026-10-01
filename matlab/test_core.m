function test_core()
%TEST_CORE Analytic geometry, planning failure and kinematic regression tests.
root=fileparts(fileparts(mfilename('fullpath')));
[scene,c]=app_load(fullfile(root,'data','benchmark_005.txt'),fullfile(root,'data','paper_parameters.txt'));
scene.bounds=[-10 40 -10 10];scene.start=[0 0 0 0];scene.goal=[25 0 0 0];
scene=rectangle(scene,-c.rear_overhang,c.length-c.rear_overhang,0,c.width/2);
rate=app_geometry('rates',scene.start,scene,c);assert(max(abs(rate-[1 0]))<1e-9);
assert(app_geometry('collision',scene.start,scene,c));
scene=rectangle(scene,3,4,-5,5);assert(app_geometry('collision',scene.start,scene,c));
scene=rectangle(scene,10,11,-10,10);assert(isempty(app_astar(scene,c)));
r=app_plan(scene,c);assert(strcmp(r.status,'no_astar_route'));
scene=rectangle(scene,10,12,-1,1);route=app_astar(scene,c);assert(max(abs(route(:,2)))>=2);
scene.triangles=zeros(0,6);scene.boxes=zeros(0,4);r=app_plan(scene,c);assert(r.success);
assert(abs(r.sample.dense(1,1)-scene.start(1))<1e-9);
assert(all(abs(r.sample.dense(:,2))<1e-9));
turn=app_sample([0 0;8 0;12 6;20 12],scene.start,[20 12 .5 0],c,true);assert(turn.valid);
assert(max(abs(turn.dense(:,4)))<=c.max_steer+1e-12);
assert(max(abs(diff(turn.dense(:,4))))<=c.max_steer_rate*c.simulation_dt/c.integration_steps+1e-12);
assert(max(abs(turn.dense(:,4)))>.1);
folded=[0 0;1 0;1 1;0 1;0 2];seg=app_segments(folded,[0 0 1 0 0],1.5,1.5);assert(isequal(seg,[1 5]));
carrots=[0 0;2 0;2 2;0 2];assert(app_matched_carrot(carrots,4,3.9)==2);
assert(app_matched_carrot(carrots,4,0)==4);assert(app_matched_carrot(carrots,4,100)==1);
scene.start(1)=-10;r=app_plan(scene,c);assert(strcmp(r.status,'invalid_endpoint'));
fprintf('All MATLAB core tests passed\n');
end
function s=rectangle(s,x0,x1,y0,y1)
s.triangles=[x0 y0 x1 y0 x1 y1;x0 y0 x1 y1 x0 y1];
s.boxes=repmat([x0 x1 y0 y1],2,1);
end
