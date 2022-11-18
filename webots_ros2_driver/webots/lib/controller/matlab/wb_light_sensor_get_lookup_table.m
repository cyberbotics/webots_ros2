function wb_light_sensor_get_lookup_table(tag)
% Usage: wb_light_sensor_get_lookup_table(tag)
% Matlab API for Webots
% Online documentation is available <a href="https://www.cyberbotics.com/doc/reference/lightsensor">here</a>

size = calllib('libController', 'wb_light_sensor_get_lookup_table_size', tag);
pointer = calllib('libController', 'wb_light_sensor_get_lookup_table', tag);
setdatatype(pointer, 'doublePtr', 3 * size, 1);
result = get(pointer, 'Value');
