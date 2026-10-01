function plot_demo(result,sceneFile,parameterFile)
%PLOT_DEMO Display an actual successful run with native MATLAB graphics.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<2, sceneFile=fullfile(root,'data','benchmark_005.txt'); end
if nargin<3, parameterFile=fullfile(root,'data','paper_parameters.txt'); end
assert(result.success,'Only successful results are plotted as valid paths');
[scene,c]=app_load(sceneFile,parameterFile); p=result.sample.dense;
figure('Color','w'); subplot(1,2,1); hold on;
t=scene.triangles; patch(t(:,1:2:5)',t(:,2:2:6)',[.8 .82 .85],'EdgeColor','none');
guide=plot(result.initial_route(:,1),result.initial_route(:,2),'--','Color',[.85 .6 .15]);
path=plot(p(:,1),p(:,2),'Color',[0 .45 .6],'LineWidth',1.5);
stride=max(1,round(4/(c.speed*c.simulation_dt/c.integration_steps)));
for i=1:stride:size(p,1)
    q=app_geometry('footprint',p(i,:),c); q(end+1,:)=q(1,:);
    plot(q(:,1),q(:,2),'Color',[.2 .65 .8],'LineWidth',.5);
end
axis equal; xlim(scene.bounds(1:2)); ylim(scene.bounds(3:4)); xlabel('x (m)'); ylabel('y (m)');
legend([guide,path],{'A* guide','APP path'},'Location','southwest'); title('Adaptive Pure Pursuit');
subplot(1,2,2); distance=(0:size(p,1)-1)'*c.speed*c.simulation_dt/c.integration_steps;
plot(distance,tan(p(:,4))/c.wheelbase,'LineWidth',1.2); hold on;
yline(tan(c.max_steer)/c.wheelbase,'--'); yline(-tan(c.max_steer)/c.wheelbase,'--');
xlabel('Travelled distance (m)'); ylabel('Curvature (1/m)'); grid on; title('Steering-limited curvature');
end
