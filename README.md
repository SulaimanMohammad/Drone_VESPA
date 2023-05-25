# Drone_VESPA

## How to find tagret in the region of the drone
The drone will scan the region by taking a hexagon path with length depends on the camera setting.
For example based on the focal length and the high of the camera, the covrage will be calculate and define the path of the drone.

```bash
        scan_hexagon(vehicle-object, VESPA-object, orgin-hex-length ,camera_image_width)
```

- It will start by calculating the distance between two successtive hexagon that confined between them the size of the image that can be captured by the drone
- calcule Distance/2 beause the drone should be in th middle.
- scan the first region
    - go to the orginal vertex V1
    - go down on (y) by distance/2 to V'1= (V1-distance/2)
    - move on the path that form hexagon
    - finish the loop
- scan the second region
    - from the last position (v'1) go by distance/2 go to v''1=( V'1-distance/2)
    - move on the path and scan a hexagon
- repeat until you can't subtract (distance/2) which means no more regions need to be scanned
![Alt text](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/.exp/coverage.png)
