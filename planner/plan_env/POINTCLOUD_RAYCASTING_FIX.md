# Point Cloud Raycasting Fix

## Problem
The exploration manager's occupancy grid was not detecting any free space when using point cloud data from depth sensors.

## Root Cause
The raycasting implementation in `sdf_map.cpp` had issues with:
1. Incorrect raycasting logic for marking free space
2. Missing point cloud preprocessing

## Solution
Fixed the raycasting logic to properly mark free space along the ray from camera to obstacle points.

## Testing
Verified with simulation point cloud data showing proper free/occupied space detection.
