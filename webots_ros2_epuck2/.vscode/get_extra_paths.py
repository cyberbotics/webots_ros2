import glob

for x in glob.glob('/home/lukic/ros2_eloquent/install/*/lib/python3.8/site-packages'):
    print('"{}",'.format(x))
