function export_benchmark(sourceFile,outputFile)
%EXPORT_BENCHMARK Convert the original MAT scene to a shared triangle union.
s=load(sourceFile,'old_params'); p=s.old_params; region=polyshape();
for j=1:numel(p.environment.obs)
    o=p.environment.obs(j); region=union(region,polyshape(o.x,o.y,'Simplify',true));
end
tr=triangulation(region); v=tr.Points; cells=tr.ConnectivityList;
f=fopen(outputFile,'w'); assert(f>=0,'Cannot write scene'); cleanup=onCleanup(@() fclose(f));
fprintf(f,'%.17g %.17g %.17g %.17g\n',p.environment.xmin,p.environment.xmax,p.environment.ymin,p.environment.ymax);
fprintf(f,'%.17g %.17g %.17g\n',p.task.x0,p.task.y0,p.task.theta0);
fprintf(f,'%.17g %.17g %.17g\n',p.task.xf,p.task.yf,p.task.thetaf);
fprintf(f,'%d\n',size(cells,1));
for j=1:size(cells,1)
    q=v(cells(j,:),:);
    if det([q(2,:)-q(1,:);q(3,:)-q(1,:)])<0, q=q([1 3 2],:); end
    fprintf(f,'%.17g %.17g %.17g %.17g %.17g %.17g\n',q');
end
end
