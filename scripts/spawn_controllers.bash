#!/bin/bash

rosservice call /controller_manager/load_controller "name: 'sphere_axis0'"
rosservice call /controller_manager/load_controller "name: 'sphere_axis1'"
rosservice call /controller_manager/load_controller "name: 'sphere_axis2'"
rosservice call /controller_manager/switch_controller "start_controllers: ['sphere_axis0','sphere_axis1','sphere_axis2']
stop_controllers: ['sphere_axis0','sphere_axis1','sphere_axis2']
strictness: 1"
