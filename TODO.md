docker compose build --build-arg GIT_PAT=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 --build-arg NETWORK_INTERFACE=eno2


SLAM algoritmus: Point LiDAR Inertial Odometry (Point-LIO) https://github.com/hku-mars/Point-LIO , https://advanced.onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459
SLAM-ben futó Kálmán szűrő: Iterated Kalman Filters on Manifolds (IkFoM) https://github.com/hku-mars/IKFoM
Point-LIO ROS2-be átírt változatavan felhasználva https://github.com/dfloreaa/point_lio_ros2
LiDAR pózt beállító segédprogram: transform_sensors https://github.com/jizhang-cmu/autonomy_stack_go2
"Body" pont pályáját és mocap pályát megjelenítő segédprogram: trajectory_bridge

+-2cm pontosság

Mérések több különböző kezdőpontból indultak.
Pontosság csökkent a kezdőpontból való távolság szerint
Statiku