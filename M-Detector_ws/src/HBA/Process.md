# Point Cloud Map Post-process
## Data Structure
``` bash
.
├── pcd
│   ├── 0.pcd
│   └── 1.pcd
└── pose.json
# pose.json每一行是：tx ty tz qw qx qy qz，与pcd顺序对应
```
## PGO
``` bash
roslaunch hba hba.launch
```
Params in [hba.launch](./launch/hba.launch),
- `data_path` 数据根目录（pcd和json的上一级）

## HBA
``` bash
roslaunch hba loopClosure_offline.launch
```
Params in [loopClosure_offline.launch](./launch/loopClosure_offline.launch),
- `iter_num` 迭代次数，一般4-5次就收敛了。如果写的是0，则会每次需要按enter才会继续。
- `thread_num` 可以分给HBA的CPU线程数
- `data_path` 数据根目录（pcd和json的上一级）

## Vis
``` bash
roslaunch hba view_map.launch
```
Params in [view_map.launch](./launch/view_map.launch),
- `data_path` 数据根目录（pcd和json的上一级）