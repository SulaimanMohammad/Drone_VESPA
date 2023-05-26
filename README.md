# Drone_VESPA

## How to find tagret in the region of the drone
The drone will scan the region by taking a hexagon path with length depends on the camera setting.
For example based on the focal length and the high of the camera, the covrage will be calculate and define the path of the drone.

```bash
        scan_hexagon(vehicle-object, VESPA-object, orgin-hex-length ,camera_image_width, scan_time)
```

- It will start by calculating the distance between two successive hexagons that confine between them the size of the image that can be captured by the drone
- calcule Distance/2 beause the drone should be in the middle.
- scan the first region
    - go north (y) to V'1= (V1-distance/2)
    - move on the path that form hexagon
    - finish the loop
- scan the second region
    - from the last position (v'1) go down by distance to v''1=( V'1-distance)
    - move on the path and scan a hexagon
    - Note: the reason of moving by "distance" not "distance/2" because (distance/2) was used to go to the path of the drone, then the next path is on "distane" as you see in the fig.
- repeat until can't subtract (distance) which means no more regions need to be scanned.
- suppose the last hexagon was the one with (v'''1), then the drone can back to the original center
by going south (a-distance*i+distance/2)
![Alt text](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/.exp/coverage.png)
