#!/bin/sh

echo "Waiting for simulations to be ready"
rosservice call /gazebo/get_physics_properties "{}"
sleep 20

echo "Spawning Assembly..."
roslaunch fetchit_challenge assembly_spawn_simple.launch &
