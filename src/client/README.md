```
docker compose up -d --force-recreate orin
```
 ✔ Container orin  Started

```
docker compose exec orin bash -lc " \
  source /opt/ros/humble/setup.bash && \
  cd /workspace && \
  colcon build --symlink-install \
"
```

    Starting >>> client  
    Starting >>> robot_msgs
    Finished <<< client [1.40s]                                                
    Finished <<< robot_msgs [9.86s]                     

    Summary: 2 packages finished [10.1s]

    
```bash
docker compose restart orin
```
- ✔ Container orin  Started    
```                          
docker compose logs -f orin
```
- orin  | Workspace not built. Run: docker compose run --rm orin bash
- orin  | [INFO] [launch]: All log files can be found below /home/cthsu/.ros/log/-  
- 2025-11-10-13-06-58-793417-orin-42
- orin  | [INFO] [launch]: Default logging verbosity is set to INFO
- orin  | [INFO] [launch.user]: Starting Orin client node, namespace: 
- orin  | [INFO] [component_container-1]: process started with pid [53]
- orin  | [INFO] [republish-2]: process started with pid [55]
