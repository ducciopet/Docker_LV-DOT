# RViz Topic Mapping Analysis

## Summary
The rviz configuration has **correct topic mappings** for all enabled displays. Topics are properly namespaced under `/onboard_detector/` as expected.

---

## ✅ ENABLED DISPLAYS (Should be visible)

| Display Name | Topic | Type | Status |
|---|---|---|---|
| Dynamic Obstacles BBoxes | `/onboard_detector/dynamic_bboxes_markers` | MarkerArray | ✅ Correctly mapped |
| Dynamic Pointcloud | `/onboard_detector/dynamic_point_cloud` | PointCloud2 | ✅ Correctly mapped |
| Raw LiDAR Cloud | `/onboard_detector/raw_lidar_point_cloud` | PointCloud2 | ✅ Correctly mapped |
| RGB Image | `/camera/color/image_raw` | Image | ⚠️ External sensor |
| Detected RGB Image | `/onboard_detector/detected_color_image` | Image | ✅ Correctly mapped |

---

## ⚠️ POTENTIAL ISSUES

### 1. **Robot Pose** Display
- **Configured Topic**: `/mavros/local_position/pose`
- **Status**: ❌ NOT published by detector
- **Issue**: This topic comes from MAVROS (external drone/vehicle driver)
- **Solution**: Either:
  - Disable this display if not using real drone feedback
  - Or provide pose from your vehicle's position source
- **Currently**: Enabled but will show "No Transform" error if topic doesn't exist

### 2. **Camera Feeds** (RGB Image)
- **Topic**: `/camera/color/image_raw`
- **Status**: ⚠️ Depends on rosbag/camera setup
- **Issue**: If playing from rosbag, camera topics must be recorded in bag
- **Check**: Run `ros2 bag info <your_bag>.db3` to see available topics
- **Currently**: Enabled with Unreliable QoS setting (correct for image streams)

---

## ✅ DISABLED BUT CORRECTLY CONFIGURED DISPLAYS

All these displays have valid topic mappings and can be enabled as needed:

| Display Name | Topic | Type |
|---|---|---|
| dbscan_bboxes | `/onboard_detector/dbscan_bboxes` | Detection2DArray - MarkerArray |
| U-depth BBoxes | `/onboard_detector/uv_bboxes` | Detection2DArray - MarkerArray |
| Visual BBoxes | `/onboard_detector/visual_bboxes` | Detection2DArray - MarkerArray |
| Lidar BBoxes | `/onboard_detector/lidar_bboxes` | Detection2DArray - MarkerArray |
| Filtered BBoxes Before YOLO | `/onboard_detector/filtered_before_yolo_bboxes` | Detection2DArray - MarkerArray |
| Filtered BBoxes | `/onboard_detector/filtered_bboxes` | Detection2DArray - MarkerArray |
| Filtered Pointcloud | `/onboard_detector/filtered_point_cloud` | PointCloud2 |
| Tracked BBoxes | `/onboard_detector/tracked_bboxes` | Detection2DArray - MarkerArray |
| Velocity Info | `/onboard_detector/velocity_visualizaton` | MarkerArray |
| Historic Trajectory | `/onboard_detector/history_trajectories` | MarkerArray |

---

## 📊 ALL PUBLISHED TOPICS FROM DETECTOR

The detector publishes these topics automatically:

### Bounding Box Topics (Detection2DArray)
- `/onboard_detector/uv_bboxes`
- `/onboard_detector/dbscan_bboxes`
- `/onboard_detector/visual_bboxes`
- `/onboard_detector/lidar_bboxes`
- `/onboard_detector/filtered_before_yolo_bboxes`
- `/onboard_detector/filtered_bboxes`
- `/onboard_detector/tracked_bboxes`
- `/onboard_detector/dynamic_bboxes` (2D detection array)

### 3D Visualization (MarkerArray)
- `/onboard_detector/dynamic_bboxes_markers` ⭐ NEW - 3D wireframe boxes
- `/onboard_detector/history_trajectories`
- `/onboard_detector/velocity_visualizaton`

### Point Cloud Topics (PointCloud2)
- `/onboard_detector/filtered_depth_cloud`
- `/onboard_detector/lidar_clusters`
- `/onboard_detector/filtered_point_cloud`
- `/onboard_detector/dynamic_point_cloud`
- `/onboard_detector/raw_dynamic_point_cloud`
- `/onboard_detector/downsampled_point_cloud`
- `/onboard_detector/raw_lidar_point_cloud`

### Image Topics
- `/onboard_detector/detected_color_image` (from detector C++ node)
- `yolo_detector/detected_image` (from YOLO Python node)

---

## 🔍 HOW TO VERIFY TOPICS IN RUNNING SYSTEM

```bash
# List all active topics
ros2 topic list

# Check what's publishing to a specific topic
ros2 topic info /onboard_detector/dynamic_bboxes_markers

# Echo topic data (first message only)
ros2 topic echo /onboard_detector/dynamic_bboxes_markers --once --max-wait 5

# Check publishing rate
ros2 topic hz /onboard_detector/dynamic_bboxes_markers
```

---

## ✅ CONFIGURATION STATUS

- **Fixed Frame**: `map` (must have TF transforms from detector)
- **Global QoS for Images**: `Unreliable: true`, `Queue Size: 10` ✅ Correct
- **Grid/Axes**: Enabled and visible
- **Namespace Filter in MarkerArray**: `box3D` filter set correctly

---

## RECOMMENDATIONS

1. ✅ **Primary Visualization**: Current setup is correct for seeing:
   - Grid and coordinate axes
   - 3D bounding boxes around obstacles
   - Point cloud data (LiDAR)
   - Detected objects (color image with annotations)

2. ⚠️ **Before Running**: Verify these exist in your rosbag:
   - `/camera/color/image_raw` (camera feed) 
   - `/onboard_detector/*` topics being published
   
3. 📌 **Optional Enhancements**:
   - Enable disabled bbox displays to see detection pipeline stages
   - Check velocity visualization if motion data is needed
   - Enable trajectory history to see tracking over time

4. 🔧 **If Nothing Shows Up**:
   - Check rviz Global Options > Fixed Frame is valid (no "Fixed Frame Error")
   - Verify topics are actually publishing: `ros2 topic list`
   - Check rviz error log (top-right corner) for subscription failures
   - Ensure `/tf_static` is publishing required frames
