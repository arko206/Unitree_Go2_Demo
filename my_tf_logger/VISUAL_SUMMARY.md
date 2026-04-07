# Visual Summary of Deque Implementation

## Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                      TFLogger Node (ROS2)                          │
│                                                                      │
│  Listens to TF Frames:                                              │
│  ├─ camera → object (Tag-T1)                                        │
│  ├─ camera → obstacle_foam                                          │
│  ├─ camera → goal_foam                                              │
│  └─ camera → base_link                                              │
│                                                                      │
└─────────────────────┬───────────────────────────────────────────────┘
                      │
                      │ tick() method every 100ms (10Hz)
                      │
        ┌─────────────┴──────────────┐
        │  Extract 4 Transformations │
        │  & Calculate Matrices      │
        │  & Convert to RPY          │
        └─────────────┬──────────────┘
                      │
        ┌─────────────┴────────────────────────────────────────┐
        │  For Each Transformation:                            │
        │  1. Extract translation [x,y,z]                      │
        │  2. Extract rotation [r,p,y]                         │
        │  3. Get previous reading                             │
        │  4. Calculate 3 error metrics                        │
        │  5. Append to deque                                  │
        │  6. Store current as "previous"                      │
        └─────────────┬────────────────────────────────────────┘
                      │
        ┌─────────────┴──────────────────────────────┐
        │                                            │
        │   4 DEQUES (Circular Buffers)             │
        │   Each with capacity of 1000 entries      │
        │                                            │
        │  ┌─ deque_camera_to_object1   ──────────┐ │
        │  │  [Entry1][Entry2]...[Entry1000]      │ │
        │  └─────────────────────────────────────┘ │
        │                                            │
        │  ┌─ deque_camera_to_object2   ──────────┐ │
        │  │  [Entry1][Entry2]...[Entry1000]      │ │
        │  └─────────────────────────────────────┘ │
        │                                            │
        │  ┌─ deque_camera_to_object3   ──────────┐ │
        │  │  [Entry1][Entry2]...[Entry1000]      │ │
        │  └─────────────────────────────────────┘ │
        │                                            │
        │  ┌─ deque_camera_to_baselink  ──────────┐ │
        │  │  [Entry1][Entry2]...[Entry1000]      │ │
        │  └─────────────────────────────────────┘ │
        │                                            │
        └────────────────────────────────────────────┘
                      │
        ┌─────────────┴───────────────────┐
        │                                 │
    Access via:                    Export via:
    ├─ get_deque_data()           ├─ CSV files
    ├─ print_deque_summary()      ├─ JSON
    └─ direct deque access        ├─ Pickle
                                   └─ Custom
```

## Deque Entry Structure (Detailed)

```
ONE DEQUE ENTRY = Dictionary with 7 Keys
═══════════════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────────────┐
│ 'timestamp': '2026-03-19 14:30:45'                                  │
│ Type: str (ISO format)                                              │
│ Content: Human-readable timestamp of reading                        │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ 'transform_matrix':                                                 │
│ ┌─ 4x4 Numpy Array ─────────────────────────────────────────────┐  │
│ │  [R00  R01  R02  Tx ]                                         │  │
│ │  [R10  R11  R12  Ty ]  (R = Rotation 3x3, T = Translation)   │  │
│ │  [R20  R21  R22  Tz ]                                         │  │
│ │  [0    0    0    1  ]                                         │  │
│ └──────────────────────────────────────────────────────────────┘  │
│ Type: numpy.ndarray(4,4)                                            │
│ Content: Complete 4x4 homogeneous transformation matrix             │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ 'translation': [0.5, 0.1, 0.3]                                      │
│ Type: list of 3 floats                                              │
│ Content: [x, y, z] in meters                                        │
│ Range: [-∞, +∞] (depends on your coordinate system)                │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ 'rotation_rpy': [0.785, 0.0, 1.571]                                 │
│ Type: list of 3 floats                                              │
│ Content: [roll, pitch, yaw] in radians                              │
│ Range: [-π, +π] for each component                                  │
│ Formula: XYZ Euler angles (intrinsic rotations)                     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ 'translation_error': 0.002                                           │
│ Type: float or None                                                 │
│ Content: Euclidean distance error from previous reading             │
│ Calculation: sqrt((x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²)                │
│ First Entry: None (no previous to compare)                          │
│ Subsequent: Always a float value ≥ 0                               │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ 'rotation_error': 0.001                                              │
│ Type: float or None                                                 │
│ Content: Frobenius norm error of rotation matrix difference         │
│ Calculation: ||R₂ - R₁||_F = sqrt(Σ(R₂ᵢⱼ - R₁ᵢⱼ)²)              │
│ First Entry: None (no previous to compare)                          │
│ Subsequent: Always a float value ≥ 0                               │
│ Interpretation: 0 = no rotation change, larger = more rotation      │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ 'rpy_error': 0.0015                                                  │
│ Type: float or None                                                 │
│ Content: Euclidean distance error of RPY values                     │
│ Calculation: sqrt((r₂-r₁)² + (p₂-p₁)² + (y₂-y₁)²)                │
│ First Entry: None (no previous to compare)                          │
│ Subsequent: Always a float value ≥ 0                               │
│ Interpretation: Amount of change in roll/pitch/yaw                  │
└─────────────────────────────────────────────────────────────────────┘
```

## Error Calculation Visualization

```
READING SEQUENCE
════════════════════════════════════════════════════════════════════════

Reading 1 (t=0.0s)
┌─────────────────────────────┐
│ timestamp: '14:30:45.000'   │
│ translation: [1.0, 2.0, 3.0]│
│ rotation_rpy: [0.1, 0.2, 0.3]│
│ translation_error: None     │  ← No previous to compare
│ rotation_error: None        │  ← No previous to compare
│ rpy_error: None             │  ← No previous to compare
└─────────────────────────────┘
              │
              │ Store as "previous"
              │
Reading 2 (t=0.1s)
┌─────────────────────────────┐
│ timestamp: '14:30:45.100'   │
│ translation: [1.05, 2.01, 3.02]│
│ rotation_rpy: [0.11, 0.19, 0.31]│
│                              │
│ translation_error:          │
│   = ||(1.05-1.0, 2.01-2.0, 3.02-3.0)||
│   = ||(0.05, 0.01, 0.02)||
│   = √(0.05² + 0.01² + 0.02²)
│   = 0.0539 meters
│                              │
│ rotation_error:             │
│   = ||R₂ - R₁||_F
│   (based on rotation matrix difference)
│   = 0.00234
│                              │
│ rpy_error:                  │
│   = ||(0.11-0.1, 0.19-0.2, 0.31-0.3)||
│   = ||(0.01, -0.01, 0.01)||
│   = √(0.01² + 0.01² + 0.01²)
│   = 0.01732 radians
└─────────────────────────────┘
              │
              │ Store as "previous"
              │
Reading 3 (t=0.2s)
┌─────────────────────────────┐
│ (similar calculation...)    │
│ translation_error: 0.0412   │
│ rotation_error: 0.00198     │
│ rpy_error: 0.01045          │
└─────────────────────────────┘
```

## Memory Layout

```
DEQUE MEMORY MANAGEMENT
════════════════════════════════════════════════════════════════════════

Initial State (Empty)
├─ deque_camera_to_object1: []
├─ deque_camera_to_object2: []
├─ deque_camera_to_object3: []
└─ deque_camera_to_baselink: []

After Running for 5 seconds at 10Hz (50 readings)
├─ deque_camera_to_object1: [Entry1, Entry2, ..., Entry50]
├─ deque_camera_to_object2: [Entry1, Entry2, ..., Entry50]
├─ deque_camera_to_object3: [Entry1, Entry2, ..., Entry50]
└─ deque_camera_to_baselink: [Entry1, Entry2, ..., Entry50]

When reaching 1000 entries (100 seconds of data at 10Hz)
├─ deque_camera_to_object1: [Entry1, Entry2, ..., Entry1000]  (FULL)
├─ deque_camera_to_object2: [Entry1, Entry2, ..., Entry1000]  (FULL)
├─ deque_camera_to_object3: [Entry1, Entry2, ..., Entry1000]  (FULL)
└─ deque_camera_to_baselink: [Entry1, Entry2, ..., Entry1000] (FULL)

After 100 seconds + 1 reading (Circular behavior)
├─ deque_camera_to_object1: [Entry2, Entry3, ..., Entry1001]  (Entry1 removed)
├─ deque_camera_to_object2: [Entry2, Entry3, ..., Entry1001]  (Entry1 removed)
├─ deque_camera_to_object3: [Entry2, Entry3, ..., Entry1001]  (Entry1 removed)
└─ deque_camera_to_baselink: [Entry2, Entry3, ..., Entry1001] (Entry1 removed)
                                ↑                       ↑
                        Oldest removed      Newest added
```

## Transformation Timeline

```
WHAT DATA IS STORED FOR EACH TRANSFORMATION
════════════════════════════════════════════════════════════════════════

Camera → Object-1
═══════════════════════════════════════════════════════════════════════
100 sec of continuous tracking at 10Hz = 1000 entries
├─ Each entry has complete 4x4 transformation matrix
├─ Translation trajectory (x,y,z) over 100 seconds
├─ Rotation trajectory (r,p,y) over 100 seconds  
└─ 999 error measurements (1st entry has None)

Useful for:
✓ Reconstructing movement history of Object-1
✓ Analyzing jitter/noise in measurements
✓ Detecting when object moves vs. stays still
✓ Velocity estimation (from consecutive errors)

Same for Camera → Object-2, Camera → Object-3, Camera → Base-Link
```

## Code Location Reference

```
File: /home/arka/Desktop/ros2_ws/src/my_tf_logger/my_tf_logger/demo_base_log.py

Lines 1-15:     Imports (including deque)
Lines 53-80:    Error calculation functions
Lines 131-150:  Deque initialization in __init__
Lines 496-636:  Data collection and deque appending in tick()
Lines 644-685:  Helper methods (get_deque_data, print_deque_summary)
Lines 688-697:  Main function
```

## Before vs. After Comparison

```
BEFORE (Text Files - Problematic)
═════════════════════════════════════════════════════════════════════
Problem 1: File Corruption
├─ File blackening after many updates
├─ Text becomes unreadable
└─ Data becomes unreliable

Problem 2: Slow I/O
├─ Disk access is slow
├─ Multiple file handles open
└─ Network drive access can timeout

Problem 3: No Error Tracking
├─ Manual calculation required
└─ No structured error metrics


AFTER (Deques - Improved)
═════════════════════════════════════════════════════════════════════
Benefit 1: Reliable Data
├─ Data in RAM, no corruption
├─ Perfectly structured
└─ Always readable

Benefit 2: Fast Access
├─ O(1) access time
├─ No disk I/O needed
└─ Always available

Benefit 3: Automatic Error Calculation
├─ 3 error metrics per reading
├─ Automatic tracking
└─ Ready for analysis
```
