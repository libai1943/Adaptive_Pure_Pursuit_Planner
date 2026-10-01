function result = run_demo(sceneFile,outputDirectory,parameterFile)
%RUN_DEMO Run the native MATLAB APP planner; no toolboxes or MEX required.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<1, sceneFile=fullfile(root,'data','benchmark_005.txt'); end
if nargin<2, outputDirectory=fullfile(root,'output','matlab'); end
if nargin<3, parameterFile=fullfile(root,'data','paper_parameters.txt'); end
[scene,c]=app_load(sceneFile,parameterFile);
timer=tic; result=app_plan(scene,c); seconds=toc(timer);
app_write_result(result,scene,c,outputDirectory);
fprintf('%s, outer iterations=%d, runtime=%.3f s\n',result.status,size(result.history,1),seconds);
end
