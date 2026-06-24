# DJI Integration Notes

当前主线：

```text
ROS2 区域覆盖规划
-> data/output/uav_waypoints.json
-> DJI Waypoint 3.0 / WPML-KMZ 航线任务文件
-> DJI Pilot 2 / MSDK App 上传并执行
```

## 系统边界

- 当前系统不直接控制无人机底层飞控。
- 当前系统只负责生成 DJI 航线任务文件。
- 航线上传、执行、暂停、恢复、停止交给 DJI MSDK / App 完成。
- 高度第一版统一使用 `relative_to_takeoff`，写入 WPML 时对应 `relativeToStartPoint`。
- 坐标第一版统一使用 `WGS84`。
- 速度第一版统一使用 `global_speed_mps` 和 `waypoint_speed_mps`。
- 第一版不写复杂动作、不写拍照动作、不写云台动作。

## ROS2 侧输出

保留现有中间航点文件：

```text
data/output/uav_waypoints.csv
data/output/uav_waypoints.json
```

字段：

```text
index
latitude
longitude
altitude_m
altitude_mode
speed_mps
coordinate_frame
```

## DJI 转换器

转换模块：

```text
sanitation_navigation.uav_waypoints_to_dji_mission_converter
```

命令：

```bash
cd /home/ubuntu/bl/workspace/robot-dog
source setup_env.sh

python scripts/uav_waypoints_to_dji_mission_converter.py
```

输入：

```text
data/output/uav_waypoints.json
```

输出：

```text
data/output/dji_mission_draft.json
data/output/dji_mission.kmz
data/output/dji_mission_validation.log
```

`dji_mission_draft.json` 用于检查字段是否正确。`dji_mission.kmz` 是给 DJI App / MSDK 上传执行的正式航线任务文件。

## Draft 字段

```json
{
  "mission_name": "uav_area_patrol",
  "mission_type": "DJI_WAYPOINT_3_0",
  "aircraft_model": "DJI Matrice 4E",
  "coordinate_frame": "WGS84",
  "altitude_mode": "relative_to_takeoff",
  "finish_action": "go_home",
  "global_speed_mps": 5.0,
  "wayline_id": 0,
  "waypoints": [
    {
      "index": 0,
      "latitude": 0.0,
      "longitude": 0.0,
      "execute_height_m": 30.0,
      "waypoint_speed_mps": 5.0
    }
  ]
}
```

字段映射：

```text
index                -> wpml:index
latitude, longitude  -> Point/coordinates
execute_height_m     -> wpml:height / wpml:executeHeight
waypoint_speed_mps   -> wpml:waypointSpeed
global_speed_mps     -> wpml:autoFlightSpeed / wpml:globalTransitionalSpeed
altitude_mode        -> wpml:heightMode / wpml:executeHeightMode
finish_action        -> wpml:finishAction
wayline_id           -> wpml:waylineId
```

## KMZ 结构

`dji_mission.kmz` 不是 JSON 改后缀，而是 ZIP/KMZ 包。内部结构：

```text
wpmz/template.kml
wpmz/waylines.wpml
```

`template.kml` 写入模板信息，`waylines.wpml` 写入执行航线。二者通过 `templateId=0` 关联。

## MSDK / App 执行流程

App 端流程：

1. 读取 `dji_mission.kmz`
2. `pushKMZFileToAircraft(kmzPath)`
3. 等待上传完成
4. `startMission(missionFileName)`
5. 监听航线任务状态
6. 监听当前航线 ID 和当前航点序号
7. 任务完成后按 `finish_action` 处理

接口含义：

```text
pushKMZFileToAircraft
上传 KMZ 航线任务文件到飞行器。

startMission
启动已上传的航线任务。通常传入上传后的 missionFileName。

addWaypointMissionExecuteStateListener
监听任务执行状态，例如上传、进入航线飞行、任务完成等。

addWaylineExecutingInfoListener
监听当前航线 ID 和当前航点序号。

checkValidation
检查 KMZ 文件的部分字段。
```

注意：`checkValidation(kmzPath)` 是 DJI MSDK Android 侧接口。ROS2/Python 转换器只能做本地 KMZ 结构校验，不能在服务器上直接调用 Android MSDK。App 侧应在上传前调用：

```kotlin
WPMZManager.getInstance().checkValidation(kmzPath)
```

## 参考

- DJI WPML 概览：`https://developer.dji.com/doc/cloud-api-tutorial/en/api-reference/dji-wpml/overview.html`
- DJI Template.kml：`https://developer.dji.com/doc/cloud-api-tutorial/en/api-reference/dji-wpml/template-kml.html`
- DJI Waylines.wpml：`https://developer.dji.com/doc/cloud-api-tutorial/en/api-reference/dji-wpml/waylines-wpml.html`
- DJI MSDK IWPMZManager：`https://developer.dji.com/api-reference-v5/android-api/Components/IWaypointMissionManager/IWPMZManager.html`
- DJI MSDK IWaypointMissionManager：`https://developer.dji.com/api-reference-v5/android-api/Components/IWaypointMissionManager/IWaypointMissionManager.html`
