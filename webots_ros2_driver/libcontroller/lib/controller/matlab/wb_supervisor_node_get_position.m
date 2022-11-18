function result = wb_supervisor_node_get_position(noderef)
% Usage: wb_supervisor_node_get_position(noderef)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_node_get_position', noderef);
setdatatype(obj,'doublePtr', 1, 3);
result = get(obj, 'Value');
