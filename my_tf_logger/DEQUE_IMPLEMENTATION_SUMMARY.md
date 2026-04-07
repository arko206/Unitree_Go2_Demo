# Deque Implementation Summary

## Overview
The code has been updated to replace file-based logging with in-memory deque data structures for storing transformation data. This solves the text file blackening issue and provides better data reliability.

## Key Changes

### 1. **Imports Added**
```python
from collections import deque
```

### 2. **Error Calculation Functions**
Three new utility functions have been added:

- `calculate_translation_error(prev_translation, curr_translation)` - Calculates Euclidean distance between consecutive translation vectors
- `calculate_rotation_error(prev_rotation_matrix, curr_rotation_matrix)` - Calculates Frobenius norm of rotation matrix difference
- `calculate_rpy_error(prev_rpy, curr_rpy)` - Calculates Euclidean distance for roll-pitch-yaw values

### 3. **Deque Initialization**
Four separate deques have been created in the `__init__` method:

```python
self.deque_camera_to_object1 = deque(maxlen=1000)   # Camera → Object-1
self.deque_camera_to_object2 = deque(maxlen=1000)   # Camera → Object-2
self.deque_camera_to_object3 = deque(maxlen=1000)   # Camera → Object-3
self.deque_camera_to_baselink = deque(maxlen=1000)  # Camera → Base-Link
```

Each deque stores up to 1000 most recent entries. Old entries are automatically discarded when the limit is exceeded.

### 4. **Data Structure**
Each deque entry contains:
```python
{
    'timestamp': str,              # ISO timestamp of reading
    'transform_matrix': np.array,  # 4x4 transformation matrix
    'translation': [x, y, z],      # Translation vector
    'rotation_rpy': [r, p, y],     # Roll, pitch, yaw in radians
    'translation_error': float,    # Euclidean error from previous reading (None for first)
    'rotation_error': float,       # Frobenius norm error from previous reading (None for first)
    'rpy_error': float             # RPY Euclidean error from previous reading (None for first)
}
```

### 5. **Error Tracking**
Previous transformation data is stored for each deque:
```python
self.prev_object1_data = None
self.prev_object2_data = None
self.prev_object3_data = None
self.prev_baselink_data = None
```

Errors are calculated between each consecutive reading automatically.

### 6. **File Operations Removed**
All text file writing operations have been replaced with deque operations:
- ❌ Removed: `self.output_file` writes
- ❌ Removed: `self.object_one_to_camera_frame` writes
- ❌ Removed: `self.object_one_to_robot_camera` writes
- ❌ Removed: `self.object_one_to_robot_base` writes
- ✅ Added: Deque-based in-memory storage

### 7. **Helper Methods**

#### `get_deque_data(deque_name)`
Retrieves all data from a specific deque.
- Arguments: 'camera_to_object1', 'camera_to_object2', 'camera_to_object3', or 'camera_to_baselink'
- Returns: List of deque entries

Example:
```python
data = node.get_deque_data('camera_to_object1')
for entry in data:
    print(entry['timestamp'], entry['translation'], entry['translation_error'])
```

#### `print_deque_summary(deque_name)`
Prints a human-readable summary of a deque's contents including the latest entry and error metrics.

### 8. **Benefits**
✅ **Reliability**: No file corruption due to rapid updates
✅ **Performance**: In-memory access is faster than file I/O
✅ **Error Tracking**: Automatic error calculation between consecutive readings
✅ **Flexible Storage**: Easy to export to files/databases later if needed
✅ **Real-time Access**: Data can be accessed programmatically at any time

## Accessing the Data

### From within the ROS2 environment:
```python
# Get data from a deque
camera_to_obj1_data = node.get_deque_data('camera_to_object1')

# Access the latest entry
if camera_to_obj1_data:
    latest = camera_to_obj1_data[-1]
    print(f"Timestamp: {latest['timestamp']}")
    print(f"Translation: {latest['translation']}")
    print(f"Translation Error: {latest['translation_error']}")
    print(f"Transform Matrix:\n{latest['transform_matrix']}")
```

### Print summaries:
```python
node.print_deque_summary('camera_to_object1')
node.print_deque_summary('camera_to_object2')
node.print_deque_summary('camera_to_object3')
node.print_deque_summary('camera_to_baselink')
```

## Exporting Data (Optional)

You can add a method to export deque data to files/pickle files later:
```python
import pickle

# Export to pickle file
with open('camera_to_object1_data.pkl', 'wb') as f:
    pickle.dump(list(node.deque_camera_to_object1), f)

# Import later
with open('camera_to_object1_data.pkl', 'rb') as f:
    data = pickle.load(f)
```

## Notes
- The deques are cleared when the node is restarted
- Maximum 1000 entries per deque (configurable via `maxlen` parameter)
- All errors start as `None` for the first reading
- Transformation matrices are copied to prevent reference issues
