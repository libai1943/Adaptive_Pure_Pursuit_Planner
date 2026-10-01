function app_write_result(r,scene,c,directory)
%APP_WRITE_RESULT Stable CSV schema for cross-language numerical regression.
if ~exist(directory,'dir'), mkdir(directory); end
csv(fullfile(directory,'initial_route.csv'),'x,y',r.initial_route);
n=size(r.sample.path,1);
csv(fullfile(directory,'path.csv'),'t,x,y,theta,phi,carrot_x,carrot_y', ...
    [(1:n)'*c.simulation_dt,r.sample.path,r.sample.carrots]);
n=size(r.sample.dense,1);
csv(fullfile(directory,'dense_path.csv'),'t,x,y,theta,phi', ...
    [(0:n-1)'*c.simulation_dt/c.integration_steps,r.sample.dense]);
csv(fullfile(directory,'history.csv'),'iteration,conflicts,segments',r.history);
summary=struct('success',r.success,'status',r.status,'outer_iterations',size(r.history,1), ...
    'goal_position_error_m',-1,'goal_heading_error_rad',-1);
if ~isempty(r.sample.path)
    p=r.sample.path(end,:); summary.goal_position_error_m=hypot(p(1)-scene.goal(1),p(2)-scene.goal(2));
    a=p(3)-scene.goal(3); summary.goal_heading_error_rad=abs(atan2(sin(a),cos(a)));
end
f=fopen(fullfile(directory,'summary.json'),'w'); assert(f>=0,'Cannot write summary');
cleanup=onCleanup(@()fclose(f)); fprintf(f,'%s\n',jsonencode(summary));
end

function csv(file,header,values)
f=fopen(file,'w'); assert(f>=0,'Cannot write result'); cleanup=onCleanup(@()fclose(f));
fprintf(f,'%s\n',header);
format=[repmat('%.17g,',1,size(values,2)-1),'%.17g\n']; fprintf(f,format,values');
end
