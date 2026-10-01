function [scene, c] = app_load(sceneFile, parameterFile)
%APP_LOAD Read the shared, language-independent scene and parameters.
text = fileread(parameterFile);
lines = regexp(text, '\r?\n', 'split'); c = struct();
for i = 1:numel(lines)
    line = strtrim(lines{i});
    if isempty(line) || line(1) == '#', continue; end
    pair = strsplit(line);
    assert(numel(pair)==2 && ~isfield(c,pair{1}), 'Invalid or duplicate parameter');
    value = str2double(pair{2});
    assert(isfinite(value) && value > 0, 'Invalid parameter');
    c.(pair{1}) = value;
end
required = {'wheelbase','width','length','rear_overhang','max_steer', ...
    'max_steer_rate','outer_iterations','inner_iterations','buffer_left', ...
    'buffer_right','buffer_increment','simulation_dt','speed','traceback_length', ...
    'nudge','grid_resolution','reference_spacing','lookahead','integration_steps', ...
    'extension','max_tracking_steps'};
assert(isempty(setxor(fieldnames(c),required)), 'Unknown or missing parameter');
for name = {'outer_iterations','inner_iterations','integration_steps','max_tracking_steps'}
    assert(c.(name{1}) == floor(c.(name{1})) && c.(name{1})<=1000000, 'Invalid integer parameter');
end
assert(c.length > c.rear_overhang && c.max_steer < pi/2, 'Invalid geometry/steering');
f = fopen(sceneFile,'r'); assert(f >= 0, 'Cannot read scene');
cleanup = onCleanup(@() fclose(f));
v = fscanf(f,'%f'); assert(numel(v)>=11 && all(isfinite(v)), 'Invalid scene');
n = v(11); assert(n>=0 && n==floor(n) && n<=100000 && numel(v)==11+6*n, 'Invalid triangle count');
scene.bounds = v(1:4)';
assert(v(1)<v(2) && v(3)<v(4), 'Invalid bounds');
scene.start = [v(5:7)' 0]; scene.goal = [v(8:10)' 0];
scene.triangles = reshape(v(12:end),6,[])';
p = scene.triangles;
assert(all((p(:,3)-p(:,1)).*(p(:,6)-p(:,2))-(p(:,4)-p(:,2)).*(p(:,5)-p(:,1))>1e-10), ...
    'Triangles must be nondegenerate and counterclockwise');
scene.boxes = [min(p(:,1:2:5),[],2) max(p(:,1:2:5),[],2) ...
    min(p(:,2:2:6),[],2) max(p(:,2:2:6),[],2)];
end
