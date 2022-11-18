function color = wb_supervisor_field_get_mf_color(field, index)
% Usage: wb_supervisor_field_get_mf_color(field, index)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/supervisor">here</a>

obj = calllib('libController', 'wb_supervisor_field_get_mf_color', field, index);
setdatatype(obj,'doublePtr', 1, 3);
color = get(obj, 'Value');
