# LV-DOT Framework Architecture

## Overview

The LV-DOT (Light-Weight Visual Dynamic Obstacle Tracker) framework is structured into **4 main modules** that operate in parallel with temporal synchronization. Each module runs at **30 Hz** (33 ms period).

---

## 1. DETECTION MODULE (Obstacle Detection)

### Description
This module detects obstacles using both visual data (depth camera) and LiDAR, fusing the information to obtain robust 3D bounding boxes. It also integrates YOLOv11 person detection to improve classification accuracy and handle human obstacles.

### Main Functions

#### 1.1 Visual Detection (UV Detection)
- **Function**: `dynamicDetector::uvDetect()`
- **Description**: Processes depth image to detect obstacles in UV coordinates
- **Sub-functions**:
  - `UVdetector::detect()` - Detects obstacles in depth image
  - `UVdetector::extract_3Dbox()` - Extracts 3D bounding boxes
  - `transformUVBBoxes()` - Transforms bounding boxes to world frame

#### 1.2 DBSCAN Visual Detection
- **Function**: `dynamicDetector::dbscanDetect()`
- **Description**: Uses DBSCAN clustering on point cloud from depth camera
- **Pipeline**:
  1. `projectDepthImage()` - Projects depth image into 3D point cloud
  2. `filterPoints()` - Filters out-of-range and ground points
  3. `clusterPointsAndBBoxes()` - DBSCAN clustering and bounding box extraction

#### 1.3 LiDAR Detection
- **Function**: `dynamicDetector::lidarDetect()`
- **Description**: Detects obstacles using LiDAR with DBSCAN
- **Pipeline**:
  1. `lidarDetector::lidarDBSCAN()` - DBSCAN clustering on LiDAR cloud
  2. `lidarDetector::getClusters()` - Extracts point clusters
  3. `lidarDetector::getBBoxes()` - Calculates bounding boxes from clusters
  4. Filters obstacles that are too large (> max_object_size)

#### 1.4 LiDAR-Visual Fusion
- **Function**: `dynamicDetector::filterLVBBoxes()`
- **Description**: Fuses visual and LiDAR detections using IOU (Intersection Over Union)
- **Algorithm (5 STEPS)**:

##### **STEP 1**: Fuse UV and DBSCAN Boxes
- Finds best bidirectional IOU match (UV↔DBSCAN)
- Minimum IOU threshold: `filtering_BBox_IOU_threshold` (0.2)
- Conservative strategy: takes maximum volume
- Output: `visualBBoxes_`

##### **STEP 2**: Process LiDAR Boxes
- Filters by maximum size
- Maintains cluster features (centroid, standard deviation)
- Output: `lidarBBoxes_`

##### **STEP 3**: Fuse Visual and LiDAR
- Combines visual boxes (Step 1) with LiDAR boxes (Step 2)
- Creates unified filtered boxes array
- Output: `filteredBBoxesBeforeYolo_` (saved for visualization)

##### **STEP 4**: Check YOLO Availability
```cpp
if (this->yoloDetectionResults_.detections.size() != 0) {
    // YOLO detections available → proceed to STEP 5
}
```

##### **STEP 5**: YOLO Integration and Box Refinement
**This is the critical fusion point where YOLO detections enhance the 3D bounding boxes.**

**5.1 3D → 2D Projection**
For each 3D bounding box from Visual-LiDAR fusion:
1. Transform bbox from world coordinates → color camera coordinates
2. Project 3D corners → 2D pixels using camera intrinsics
3. Create projected 2D bounding boxes (`filteredDetectionResults`)

```cpp
// Pinhole camera projection
int tlX = (fxC_ * topleft.x + cxC_ * topleft.z) / topleft.z;
int tlY = (fyC_ * topleft.y + cyC_ * topleft.z) / topleft.z;
```

**5.2 YOLO ↔ 3D Boxes Matching (2D IOU)**
For each YOLO detection:
- Calculates 2D IOU between YOLO bbox and all projected 3D bboxes
- Finds best match: `best3DBBoxForYOLO[i] = bestIdx`
- Creates mapping: `box3DToYolo[idx3D] = [yolo_indices]`

**5.3 Classification and Splitting**

**CASE 1: No YOLO match for 3D bbox**
```cpp
if (it == box3DToYolo.end()) {
    // Keep original 3D bbox without modifications
    newFilteredBBoxes.push_back(filteredBBoxesTemp[idx3D]);
}
```

**CASE 2: Single YOLO match ← 3D box (Common case)**
```cpp
if (yoloIndices.size() == 1) {
    filteredBBoxesTemp[idx3D].is_dynamic = true;  // ← MARKED DYNAMIC!
    filteredBBoxesTemp[idx3D].is_human = true;    // ← MARKED HUMAN!
    newFilteredBBoxes.push_back(filteredBBoxesTemp[idx3D]);
}
```
**⚡ Meaning**: YOLO recognized a person → automatically dynamic

**CASE 3: Multiple YOLO boxes → 1 3D bbox (Close people)**
```cpp
} else {  // yoloIndices.size() > 1
    // SPLITTING ALGORITHM
```

**Splitting Algorithm:**
1. Takes point cloud from fused 3D bbox
2. For each 3D point:
   - Projects to 2D image
   - Finds which YOLO box contains the point
   - Assigns point to closest YOLO box
3. **Splits point cloud** into sub-clouds (one per YOLO box)
4. For each sub-cloud:
   - Calculates new 3D bounding box (AABB - Axis-Aligned Bounding Box)
   - Marks as `is_dynamic = true` and `is_human = true`
   - Replaces original 3D bbox with N separate boxes

**Practical Example:**
```
Before:  1 large 3D bbox containing 2 close people
        ┌─────────────┐
        │   👤  👤    │  ← Single 3D detection
        └─────────────┘

YOLO:   2 separate detections
        ┌────┐ ┌────┐
        │ 👤 │ │ 👤 │  ← Two distinct people
        └────┘ └────┘

After:   2 separate 3D bboxes
        ┌────┐ ┌────┐
        │ 👤 │ │ 👤 │  ← Splitting successful!
        └────┘ └────┘
```

**5.4 Final Update**
```cpp
filteredBBoxesTemp = newFilteredBBoxes;  // Replace with YOLO-processed results
this->filteredBBoxes_ = filteredBBoxesTemp;  // Final output
```

#### 1.5 YOLO Person Detection (Asynchronous)
- **Function**: `dynamicDetector::yoloDetectionCB()`
- **Description**: Receives person detections from YOLOv11
- **Separate node**: `yolov11_detector_node.py` (Python)
- **Output**: Vision messages with 2D person bounding boxes
- **Interface**:
```cpp
void dynamicDetector::yoloDetectionCB(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections){
    this->yoloDetectionResults_ = *detections;  // Save in buffer
}
```

**Characteristics:**
- **ROS2 Subscriber**: Topic `yolo_detector/detected_bounding_boxes`
- **Message type**: `vision_msgs::msg::Detection2DArray` (2D person bboxes)
- **Execution**: **Asynchronous** and independent from detection timers
- **Function**: Simply updates buffer `yoloDetectionResults_` with latest message
- **Frequency**: ~30 Hz (from separate Python YOLO node)

### Main Detection Callback
```cpp
void dynamicDetector::detectionCB() {
    this->dbscanDetect();        // STEP 1: Depth camera detection
    this->uvDetect();            // STEP 2: UV detection
    this->filterLVBBoxes();      // STEP 3: LiDAR-Visual-YOLO Fusion ← KEY POINT
    this->newDetectFlag_ = true; // Signal new detection
}
```

### Timing Constants

| Parameter | Value | Description |
|-----------|-------|-------------|
| **time_step** | 0.033 s (33 ms) | Detection update period |
| **Frequency** | 30 Hz | Module execution frequency |
| **YOLO timer** | 0.033 s (33 ms) | YOLO inference period |
| **YOLO inference** | ~30 ms | Actual GPU inference time |

### Key Parameters

#### Camera
- **Depth intrinsics**: fx=385.31, fy=385.31, cx=324.80, cy=237.72
- **Color intrinsics**: fx=606.31, fy=605.93, cx=314.69, cy=252.19
- **Resolution**: 640×480 pixels
- **Depth scale**: 1000 (mm to meters)
- **Depth range**: 0.5 - 5.0 meters

#### DBSCAN Visual
- **voxel_occupied_thresh**: 5.0 points (min points for occupied voxel)
- **dbscan_min_points_cluster**: 20-40 points
  - 20: 4.0m range
  - 30: 3.5m range  
  - 40: 3.0m range
- **dbscan_search_range_epsilon**: 0.05 meters (DBSCAN search radius)
- **depth_skip_pixel**: 2 (depth image downsampling)

#### DBSCAN LiDAR
- **lidar_DBSCAN_min_points**: 10 points
- **lidar_DBSCAN_epsilon**: 0.05 meters
- **downsample_threshold**: 3500 points (threshold for gaussian downsampling)
- **gaussian_downsample_rate**: 6 (downsampling factor)

#### Filtering Heights
- **ground_height**: 0.2 meters (removes ground points)
- **roof_height**: 2.0 meters (removes points above this relative height)

#### Obstacle Dimensions
- **max_object_size**: [3.0, 3.0, 2.0] meters (x, y, z)
- **target_object_size**: [0.5, 0.5, 1.5] meters (target size if constraint enabled)

#### YOLO-3D Fusion
- **IOU threshold**: Any positive IOU accepted for matching
- **Splitting margin**: 0 pixels (can be adjusted for looser assignment)
- **Box update**: Conservative (maintains larger dimensions)

---

## 2. TRACKING MODULE (Data Association & Kalman Filter)

### Description
Associates current detections with previously tracked obstacles and applies Kalman Filter to estimate state and velocity.

### Main Functions

#### 2.1 Data Association
- **Function**: `dynamicDetector::boxAssociation(std::vector<int>& bestMatch)`
- **Description**: Finds best correspondence between current detections and tracked obstacles
- **Algorithm**:
  - Calculates weighted distance between detections and historical obstacles
  - Uses multi-dimensional features: position (3D), size (3D), point cloud centroid (3D)
  - Applies Hungarian algorithm for optimal matching
  - Matching thresholds based on distance and size difference

- **Helper**: `dynamicDetector::boxAssociationHelper(std::vector<int>& bestMatch)`
  - Calculates cost matrix for matching
  - Verifies distance and size constraints

#### 2.2 Kalman Filter & History Update
- **Function**: `dynamicDetector::kalmanFilterAndUpdateHist(const std::vector<int>& bestMatch)`
- **Description**: Applies Kalman Filter for state estimation and updates history
- **Estimated states**:
  - Position: (x, y, z)
  - Velocity: (Vx, Vy)
  - Acceleration: (Ax, Ay) [optional]
- **Pipeline**:
  1. For each matched detection: update existing filter
  2. For unmatched detections: create new filter
  3. Estimate state using current observation
  4. Update history (bounding box + point cloud + centroids)
  5. Maintains limited history size (max 100 frames)

#### 2.3 Kalman Filter Setup
- **Functions**:
  - `kalmanFilterMatrixAcc()` - Configures matrices for acceleration model
  - `getKalmanObservationAcc()` - Prepares observation vector
- **Dynamic model**: Constant Acceleration Model (CA)
  - States: [x, y, Vx, Vy, Ax, Ay]
  - Measurements: [x, y] position

#### 2.4 Feature Extraction
- **Auxiliary functions**:
  - `getPointCloudFeatures()` - Extracts features from point cloud
  - `getSizeFeatures()` - Extracts bounding box dimensions
  - `distanceBetweenBox3D()` - Calculates weighted distance between boxes

### Main Tracking Callback
```cpp
void dynamicDetector::trackingCB() {
    std::vector<int> bestMatch;
    this->boxAssociation(bestMatch);  // Data association
    
    if (bestMatch.size()) {
        this->kalmanFilterAndUpdateHist(bestMatch);  // Kalman + history update
    } else {
        // Reset history if no association
        this->boxHist_.clear();
        this->pcHist_.clear();
        this->pcCenterHist_.clear();
    }
}
```

### Timing Constants

| Parameter | Value | Description |
|-----------|-------|-------------|
| **time_step** | 0.033 s (33 ms) | Tracking update period |
| **Frequency** | 30 Hz | Module execution frequency |
| **history_size** | 100 frames | Maximum history size per obstacle |
| **kalman_filter_averaging_frames** | 10 frames | Kalman Filter averaging window |

### Key Parameters

#### Data Association
- **max_match_range**: 0.5 meters (max distance for match)
- **max_size_diff_range**: 0.5 meters (max size difference)
- **feature_weight**: [3.0, 3.0, 0.1, 0.5, 0.5, 0.05, 0.0, 0.0, 0.0]
  - Weights for: [pos_x, pos_y, pos_z, size_x, size_y, size_z, pc_center_x, pc_center_y, pc_center_z]
  - Position has maximum weight (3.0)
  - Size has medium weight (0.5)
  - Point cloud centroid has minimum weight (0.05)

#### Kalman Filter
- **kalman_filter_param**: [0.25, 0.01, 0.05, 0.05, 0.04, 0.3, 0.6]
  - Process noise and measurement noise covariance
  - Tuned to balance responsiveness and smooth tracking

#### Size Fixing
- **fix_size_history_threshold**: 10 frames
  - After 10 tracking frames, obstacle size is fixed
- **fix_size_dimension_threshold**: 0.4 (40%)
  - Size variation threshold for fixing

---

## 3. CLASSIFICATION MODULE (Dynamic/Static Classification)

### Description
Classifies tracked obstacles as dynamic or static by analyzing point cloud motion and velocity estimated by Kalman Filter.

### Main Functions

#### 3.1 Main Classification
- **Function**: `dynamicDetector::classificationCB()`
- **Description**: Classifies obstacles using multi-criteria analysis
- **3-level algorithm**:

##### CASE I: YOLO Human Detection
- If YOLO recognizes a person → **automatically DYNAMIC**
- `boxHist_[i][0].is_human == true`
- **Bypasses all other verifications**
- **⚡ Impact**: Reduces latency from 0.5s to 33ms for YOLO-detected people!

##### CASE II: History Length Check
- Verifies if history is sufficient for classification
- Minimum required: `skipFrame_ + 1` frames
- If insufficient: skip classification

##### CASE III: Force Dynamic
- If obstacle has been classified as dynamic for N consecutive frames
- **frames_force_dynamic**: 10 frames (search range)
- **frames_force_dynamic_check_range**: 30 frames (verification threshold)
- If `dynaFrames >= forceDynaFrames_` → **DYNAMIC**

#### 3.2 Point Cloud Velocity Analysis
- **Nearest Neighbor Algorithm**:
  1. Compares current point cloud vs point cloud N frames ago
  2. For each current point, finds nearest neighbor in previous frame
  3. Calculates point-to-point velocity: `V = (curr - prev) / (dt * frameGap)`
  4. Compares point velocity direction with bounding box velocity (dot product)
  5. If negative alignment: discard point
  6. If |V| > velocity threshold: increment dynamic vote

#### 3.3 Voting Mechanism
- **Vote ratio calculation**: `voteRatio = votes / numValidPoints`
- **Conditions for DYNAMIC classification**:
  1. `voteRatio >= dynaVoteThresh_` (0.8 = 80% dynamic points)
  2. `|V_kalman| >= dynaVelThresh_` (0.2 m/s)
  3. Dynamic consistency check (see below)

#### 3.4 Dynamic Consistency Check
- **Parameter**: `dynamic_consistency_threshold` = 15 frames
- **Logic**:
  - Obstacle must be voted as dynamic for 15 **consecutive** frames
  - Counts how many of last 15 frames are `is_dynamic_candidate` or `is_human` or `is_dynamic`
  - Only if all 15 frames satisfy condition → `is_dynamic = true`
- **Purpose**: Reduces false positives, ensures motion persistence

#### 3.5 Size Constraint
- **Function**: Final filtering by target dimensions
- **Parameter**: `target_constrain_size` = true/false
- **Logic**:
  - Compares dynamic obstacle dimensions with `target_object_size`
  - Tolerance: xdiff < 0.8m, ydiff < 0.8m, zdiff < 1.0m
  - Only obstacles with compatible dimensions pass filter

### Main Classification Callback
```cpp
void dynamicDetector::classificationCB() {
    std::vector<onboardDetector::box3D> dynamicBBoxesTemp;
    
    for (size_t i=0; i<this->pcHist_.size(); ++i) {
        // CASE I: YOLO human → dynamic (IMMEDIATE)
        if (this->boxHist_[i][0].is_human) {
            dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
            continue;  // ← BYPASS all other checks!
        }
        
        // CASE II: History check
        int curFrameGap = (pcHist_[i].size() < skipFrame_+1) 
                          ? pcHist_[i].size()-1 : skipFrame_;
        
        // CASE III: Force dynamic
        if (dynaFrames >= forceDynaFrames_) {
            dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
            continue;
        }
        
        // Point cloud velocity analysis + voting
        // ... (detailed algorithm above)
        
        // Dynamic consistency check
        if (dynaConsistCount == dynamicConsistThresh_) {
            dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
        }
    }
    
    // Size constraint filtering
    if (this->constrainSize_) {
        // ... filter by target_object_size
    }
    
    this->dynamicBBoxes_ = dynamicBBoxesTemp;
}
```

### Timing Constants

| Parameter | Value | Description |
|-----------|-------|-------------|
| **time_step** | 0.033 s (33 ms) | Classification period |
| **Frequency** | 30 Hz | Module execution frequency |
| **frame_skip** | 5 frames | Temporal gap for pointcloud comparison |
| **Comparison time** | 5 × 33 ms = **165 ms** | Time between compared frames |

### Key Parameters

#### Velocity Thresholds
- **dynamic_velocity_threshold**: 0.2 m/s
  - Minimum velocity to consider point dynamic
  - Equivalent to 0.72 km/h (very sensitive to slow movements)

#### Voting
- **dynamic_voting_threshold**: 0.8 (80%)
  - Minimum percentage of dynamic points for classification
  - High threshold to reduce false positives

#### Force Dynamic
- **frames_force_dynamic**: 10 frames
  - Minimum recent dynamic frames for force
  - Equivalent time: 10 × 33 ms = 330 ms
- **frames_force_dynamic_check_range**: 30 frames
  - History range analyzed for force dynamic
  - Equivalent time: 30 × 33 ms = 990 ms ≈ 1 second

#### Consistency
- **dynamic_consistency_threshold**: 15 frames
  - Consecutive frames required for definitive classification
  - Equivalent time: 15 × 33 ms = **495 ms ≈ 0.5 seconds**
  - **MINIMUM LATENCY for dynamic detection**: 0.5 seconds
  - **Exception**: YOLO-detected humans bypass this → 33 ms latency!

---

## 4. VISUALIZATION MODULE

### Description
Publishes visualizations for RViz2 and debugging. Not part of core methodology but essential for debug and tuning.

### Main Functions

#### 4.1 Visualization Callback
- **Function**: `dynamicDetector::visCB()`
- **Description**: Publishes RViz markers and images
- **Output**:
  - UV detection images
  - Color images with YOLO boxes
  - 3D bounding boxes (all stages)
  - Filtered and clustered point clouds
  - Trajectory history

#### 4.2 Publishers
- **Image Publishers**: 
  - Depth maps (UV, depth, bird view)
  - Color images with detection overlay
- **Marker Publishers**:
  - UV boxes, DBSCAN boxes, Visual boxes, LiDAR boxes
  - Filtered boxes (before/after YOLO), Tracked boxes, Dynamic boxes
- **PointCloud Publishers**:
  - Filtered depth points, LiDAR clusters
  - Dynamic points, downsampled points

### Main Visualization Callback
```cpp
void dynamicDetector::visCB() {
    this->publishUVImages();
    this->publishColorImages();
    this->publish3dBox(this->uvBBoxes_, this->uvBBoxesPub_, 0, 1, 0);  // Green
    this->publish3dBox(this->dbBBoxes_, this->dbBBoxesPub_, 1, 0, 0);  // Red
    this->publish3dBox(this->visualBBoxes_, this->visualBBoxesPub_, ...);
    this->publish3dBox(this->lidarBBoxes_, this->lidarBBoxesPub_, ...);
    this->publish3dBox(this->filteredBBoxesBeforeYolo_, this->filteredBBoxesBeforeYoloPub_, ...);
    this->publish3dBox(this->filteredBBoxes_, this->filteredBBoxesPub_, ...);
    this->publish3dBox(this->trackedBBoxes_, this->trackedBBoxesPub_, ...);
    this->publish3dBox(this->dynamicBBoxes_, this->dynamicBBoxesPub_, ...);
    // ... point cloud publishers
}
```

### Timing Constants

| Parameter | Value | Description |
|-----------|-------|-------------|
| **time_step** | 0.033 s (33 ms) | Publication period |
| **Frequency** | 30 Hz | Module execution frequency |

---

## YOLO-VISUAL-LIDAR FUSION INTERFACE

### Asynchronous Design with Buffer

**Architecture:**
```
┌──────────────────────────────────────────────────┐
│  YOLO Node (Python, async 30Hz)                  │
│  - Receives color image                          │
│  - YOLOv11 inference (~30ms)                     │
│  - Publishes Detection2DArray                    │
└────────────────┬─────────────────────────────────┘
                 │ Topic: yolo_detector/detected_bounding_boxes
                 ↓ (asynchronous)
┌────────────────▼─────────────────────────────────┐
│  yoloDetectionCB()                               │
│  - Saves to buffer: yoloDetectionResults_        │
│  - Non-blocking, returns immediately            │
└──────────────────────────────────────────────────┘
                 │
                 │ Buffer access
                 ↓
┌──────────────────────────────────────────────────┐
│  detectionCB() Timer (30Hz)                      │
│  1. dbscanDetect()                               │
│  2. uvDetect()                                   │
│  3. filterLVBBoxes() ← READS yoloDetectionResults_│
│     └─ STEP 5: YOLO Integration                  │
└──────────────────────────────────────────────────┘
```

### Key Design Principles

#### 1. Asynchronous Buffer Pattern
- YOLO callback **asynchronously** updates buffer `yoloDetectionResults_`
- Detection timer **reads** buffer when needed (STEP 5)
- **Advantage**: YOLO inference doesn't block main pipeline

#### 2. "Soft" Synchronization
- No explicit synchronization between YOLO and detection
- Uses **latest available YOLO message** at fusion time
- With both at 30Hz, typically aligned (±10-20ms)

#### 3. Hierarchical Fusion
```
Visual Detection (UV + DBSCAN) + LiDAR Detection
            ↓
    Visual-LiDAR Fusion (3D IOU)
            ↓
    3D → 2D Projection
            ↓
    YOLO Matching (2D IOU)
            ↓
    Classification + Splitting
            ↓
    filteredBBoxes_ (final output)
```

### Benefits of YOLO Integration

| Without YOLO | With YOLO |
|--------------|-----------|
| Generic 3D bbox | Bbox marked `is_human=true` |
| Dynamic classification requires 0.5s | **Instant classification** |
| Close people → 1 large bbox | **Automatic splitting** into N people |
| False positives on static objects | Reduced false positives |
| No semantic information | Semantic class (person) |

### Impact on Classification Module

In `classificationCB()`:
```cpp
// CASE I: YOLO human detection
if (this->boxHist_[i][0].is_human) {
    dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
    continue;  // ← BYPASS everything else!
}
```

**Benefit**: Obstacles marked `is_human` by YOLO **completely bypass**:
- Point cloud velocity analysis
- Voting mechanism
- Dynamic consistency check (15 frames)

**Result**: **Latency reduced from 0.5s to 33ms** for YOLO-recognized people!

### Performance and Timing

| Component | Time | Notes |
|-----------|------|-------|
| **YOLO inference** | ~30 ms | Separate Python node |
| **yoloDetectionCB()** | <1 ms | Only buffer copy |
| **filterLVBBoxes() STEP 5** | ~5-10 ms | Projection + matching + splitting |
| **Total detection** | ~20-31 ms | Includes everything |
| **YOLO overhead** | ~5-10 ms | Only when detections available |

### Why This Design is Elegant

1. **Doesn't slow down** pipeline even if YOLO is slow
2. **Drastically improves** classification for people
3. **Resolves ambiguities** (close people) that 3D sensors can't distinguish
4. **Optional**: if YOLO detects nothing, pipeline continues normally
5. **Robust**: Buffer ensures latest data always available
6. **Scalable**: Can add more semantic detectors following same pattern

---

## COMPLETE PIPELINE AND TIMING ANALYSIS

### Execution Sequence

```
┌─────────────────────────────────────────────────────────────┐
│                    CYCLE: 33 ms (30 Hz)                      │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌───────────────────────────────────────────────┐          │
│  │ 1. DETECTION MODULE                           │          │
│  │    - UV Detect (depth image)                  │          │
│  │    - DBSCAN Detect (depth pointcloud)         │          │
│  │    - LiDAR Detect (LiDAR DBSCAN)              │          │
│  │    - Filter & Fuse (LV-Boxes)                 │          │
│  │    - YOLO Integration (if available)          │          │
│  │    Duration: ~10-20 ms                        │          │
│  └───────────────────────────────────────────────┘          │
│                         ↓                                     │
│  ┌───────────────────────────────────────────────┐          │
│  │ 2. TRACKING MODULE                            │          │
│  │    - Box Association (Hungarian)              │          │
│  │    - Kalman Filter Update                     │          │
│  │    - History Update                           │          │
│  │    Duration: ~3-5 ms                          │          │
│  └───────────────────────────────────────────────┘          │
│                         ↓                                     │
│  ┌───────────────────────────────────────────────┐          │
│  │ 3. CLASSIFICATION MODULE                      │          │
│  │    - YOLO Human Check (instant)               │          │
│  │    - Point Cloud Velocity Analysis            │          │
│  │    - Voting & Consistency Check               │          │
│  │    - Dynamic Classification                   │          │
│  │    Duration: ~5-8 ms                          │          │
│  └───────────────────────────────────────────────┘          │
│                         ↓                                     │
│  ┌───────────────────────────────────────────────┐          │
│  │ 4. VISUALIZATION MODULE                       │          │
│  │    - Publish Markers                          │          │
│  │    - Publish Point Clouds                     │          │
│  │    - Publish Images                           │          │
│  │    Duration: ~2-3 ms                          │          │
│  └───────────────────────────────────────────────┘          │
│                                                               │
│  TOTAL PROCESSING: ~20-36 ms                                 │
│  MARGIN: ~-3 to +13 ms                                       │
└─────────────────────────────────────────────────────────────┘

         PARALLEL PROCESS: YOLO Detector (30 Hz)
         - Inference: ~30 ms per frame
         - Runs asynchronously in separate Python node
         - Non-blocking buffer update
```

### System Latencies

| Phase | Latency | Notes |
|-------|---------|-------|
| **Single detection** | 33 ms | 1 frame |
| **First tracking match** | 66 ms | 2 frames |
| **Classification (YOLO)** | 33 ms | Instant for humans! |
| **Classification (min)** | 495 ms | 15 frame consistency |
| **Classification (typical)** | 660-990 ms | Includes frame_skip and force dynamic check |
| **Detection → Classification** | 0.033 - 1.0 s | End-to-end latency (depends on YOLO) |

### Memory and History

| Buffer | Size | Memory (est.) |
|--------|------|---------------|
| **boxHist_** | 100 frames × N_objects | ~few KB per object |
| **pcHist_** | 100 frames × points × N_objects | ~MB scale |
| **pcCenterHist_** | 100 frames × N_objects | ~few KB |
| **Kalman Filters** | N_objects × state vector | Negligible |
| **yoloDetectionResults_** | 1 message | ~few KB |

---

## CONFIGURATION AND TUNING

### Configuration File
**Path**: `src/onboard_detector/cfg/detector_param.yaml`

### Critical Parameters for Performance

#### For Speed (Real-time Enforcement)
- Reduce `dbscan_min_points_cluster` (20 is fast, 40 is precise)
- Increase `depth_skip_pixel` (2 → 3 for more aggressive downsampling)
- Increase `gaussian_downsample_rate` (6 → 8 for LiDAR downsampling)
- Reduce `history_size` if memory is limited
- Disable YOLO if not needed (but lose instant person detection)

#### For Accuracy (Precision Optimization)
- Increase `dbscan_min_points_cluster` (favors denser clusters)
- Reduce `depth_skip_pixel` (1 for full resolution)
- Increase `dynamic_consistency_threshold` (reduces false positives)
- Reduce `dynamic_velocity_threshold` (more sensitive dynamicity detection)
- Enable YOLO for better person detection

#### For Robustness
- Increase `filtering_BBox_IOU_threshold` (0.2 → 0.3 for stricter matches)
- Increase `dynamic_voting_threshold` (0.8 → 0.9 for more certainty)
- Increase `dynamic_consistency_threshold` (15 → 20 frames)
- Keep YOLO enabled for reliable human detection

---

## ROS2 TOPICS

### Input Topics
- `/camera/depth/image_rect_raw` - Depth image (sensor_msgs/Image)
- `/camera/color/image_raw` - Color image (sensor_msgs/Image)
- `/pointcloud` - LiDAR pointcloud (sensor_msgs/PointCloud2)
- `/mavros/local_position/pose` - Robot pose (geometry_msgs/PoseStamped)
- `yolo_detector/detected_bounding_boxes` - YOLO detections (vision_msgs/Detection2DArray)

### Output Topics (Detection)
- `onboard_detector/filtered_bboxes_before_yolo` - Bounding boxes after LV fusion, before YOLO
- `onboard_detector/filtered_bboxes` - Final bounding boxes after YOLO integration
- `onboard_detector/tracked_bboxes` - Tracked bounding boxes
- `onboard_detector/dynamic_bboxes` - Classified dynamic obstacles
- `onboard_detector/dynamic_pointcloud` - Point clouds of dynamic obstacles

### Output Topics (Debug)
- `onboard_detector/uv_bboxes` - UV detection boxes
- `onboard_detector/dbscan_bboxes` - DBSCAN detection boxes
- `onboard_detector/lidar_bboxes` - LiDAR detection boxes
- `onboard_detector/visual_bboxes` - Fused visual boxes
- `onboard_detector/history_trajectory` - Trajectory history markers
- `onboard_detector/detected_color_image` - Color image with YOLO boxes overlay

### Services
- `onboard_detector/get_dynamic_obstacles` - Query for dynamic obstacles in range

---

## REFERENCES

### Original Paper
- **LV-DOT**: Light-Weight Visual Dynamic Obstacle Tracker
- Original repository: https://github.com/Zhefan-Xu/LV-DOT

### Algorithms Used
- **DBSCAN**: Density-Based Spatial Clustering of Applications with Noise
- **Hungarian Algorithm**: For optimal data association
- **Kalman Filter**: For state estimation (Constant Acceleration Model)
- **YOLOv11**: For person detection

### Supported Sensors
- **Camera**: Intel RealSense (depth + color) or equivalent
- **LiDAR**: VLP-16 or similar (3D pointcloud)
- **Localization**: MAVROS (PX4/ArduPilot) or Odometry

---

## IMPLEMENTATION NOTES

### Thread Safety
All modules use **shared state** (boxHist_, pcHist_, filters_) updated sequentially by ROS2 timers. There's no true parallelism, but sequential execution scheduled by timers.

### Sensor Synchronization
- Depth + Pose: ApproximateTime sync policy
- LiDAR + Pose: ApproximateTime sync policy
- YOLO: Independent asynchronous callback
- Color image: Independent subscriber

### Hardware Performance Requirements
- **GPU required**: NVIDIA GPU for YOLO inference
- **CPU**: Multi-core recommended for parallel processing
- **RAM**: ~2-4 GB for history buffers with 10-20 obstacles
- **Disk**: Minimal (no logging by default)

### Limitations
- **Range**: Limited by depth camera (max 5m) and LiDAR range
- **Classification latency**: Minimum 0.5s for non-YOLO dynamic classification
- **Occlusions**: May lose tracking during prolonged occlusions
- **Crowded environments**: Performance degrades with >20 simultaneous obstacles
- **YOLO dependency**: Requires GPU and adds ~5-10ms overhead
- **2D-3D projection**: Assumes reasonably accurate camera calibration

### Best Practices
1. **Always enable YOLO** for human-robot interaction scenarios
2. Tune DBSCAN parameters based on environment (indoor vs outdoor)
3. Adjust consistency thresholds based on required response time
4. Monitor processing time to ensure <33ms per cycle
5. Use RViz2 visualization for parameter tuning
6. Test with recorded bag files before deployment
