# multi-laserscan-toolbox-ros2
Customize and merge several laserscan messages into one virtual source.

Still need to take into consideration thos parameters:
   #output settings
    angle_min: 0.0
    angle_max: 6.283185307
    angle_increment: 0.017453293 #0.017453293 for 360 values resolution
    range_min: 0.0 #m
    range_max: 100.0 #m

    #advanced output settings
    auto_set: true 
    time_increment: 0.0 #s
    scan_time: 0.0 #s 

    #for each sources
    range_min: 0.0 #m
    range_max: 100.0 #m
    persistence: false
    timeout: 0.0 #0.0 seconds = all value's date are autorized

Also, 
- manage different resolution possibilities in fuseScans --> (related to angle_increment)
- compare angle min/max chosen for a sources and the one detected in the LaserScan message