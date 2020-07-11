# Project Instructions

In this project you'll implement Model Predictive Control to drive the car around the track. This time however you're not given the cross track error, you'll have to calculate that yourself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

## Simulator Misc

* A singular unity unit in the simulator is equivalent to 1 meter.
* For details on the data sent back from the server, read this file.

## Visualization

When working on the MPC project it helps to visualize both your reference path and the MPC trajectory path.

You can display these connected point paths in the simulator by sending a list of optional x and y values to the `mpc_x`, `mpc_y`, `next_x`, and `next_y` fields in the C++ main script. If these fields are left untouched then simply no path will be displayed.

The `mpc_x` and `mpc_y` variables display a line projection in green. The `next_x` and `next_y` variables display a line projection in yellow. You can display these both at the same time, as seen in the image above.

These (x,y) points are displayed in reference to the vehicle's coordinate system. Recall that the x axis always points in the direction of the car’s heading and the y axis points to the left of the car. So if you wanted to display a point 10 units directly in front of the car, you could set `next_x = {10.0}` and `next_y = {0.0}`.

Remember that the server returns waypoints using the map's coordinate system, which is different than the car's coordinate system. Transforming these waypoints will make it easier to both display them and to calculate the CTE and Epsi values for the model predictive controller.

## IPOPT Installation on Mac

When executing brew install ipopt, some Mac users have experienced the following error:

```
Listening to port 4567
Connected!!!
mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object - object was probably modified after being freed.
*** set a breakpoint in malloc_error_break to debug
```

This error can be resolved by upgrading ipopt with ``brew upgrade ipopt --with-openblas``.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.
