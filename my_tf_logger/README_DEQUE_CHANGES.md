# TFLogger Deque Implementation - Complete Guide

## Overview

Your TFLogger has been successfully updated to use in-memory **deque data structures** instead of text files for storing transformation data. This solves the text file corruption issue and provides better data reliability and performance.

## What Changed? ✨

### **Before (Old Approach)**
- ❌ Data saved to multiple text files
- ❌ Files became unreadable after many updates
- ❌ Slow file I/O operations
- ❌ No automatic error tracking

### **After (New Approach)**
- ✅ Data stored in memory using deques
- ✅ Automatic error calculation between consecutive readings
- ✅ Fast in-memory access
- ✅ Up to 1000 readings per transformation stored
- ✅ Easy export to files/pickle/JSON when needed

## File Structure

```
my_tf_logger/
├── my_tf_logger/
│   ├── __init__.py
│   ├── demo_base_log.py          # ✨ MODIFIED - Main TFLogger node with deques
│   └── deque_exporter.py         # NEW - Utility for exporting deque data
├── DEQUE_IMPLEMENTATION_SUMMARY.md    # Technical implementation details
├── DEQUE_USAGE_EXAMPLES.md           # Practical usage examples and patterns
└── README.md                         # This file
```

## Key Features

### 1️⃣ **Four Separate Deques**

Each transformation stream has its own deque with up to 1000 entries:

```python
deque_camera_to_object1      # Camera → Object-1 (Tag-T1)
deque_camera_to_object2      # Camera → Obstacle Foam
deque_camera_to_object3      # Camera → Goal Foam
deque_camera_to_baselink     # Camera → Base-Link
```

### 2️⃣ **Complete Data Storage**

Each deque entry contains:

```python
{
    'timestamp': '2026-03-19 14:30:45',
    'transform_matrix': np.array(4x4),  # Full 4x4 transformation matrix
    'translation': [x, y, z],           # Translation vector
    'rotation_rpy': [r, p, y],          # Roll, Pitch, Yaw in radians
    'translation_error': float,         # Error from previous reading
    'rotation_error': float,            # Rotation error
    'rpy_error': float                  # RPY error
}
```

### 3️⃣ **Automatic Error Calculation**

Errors are calculated automatically between consecutive readings:

- **Translation Error**: Euclidean distance between consecutive translations
- **Rotation Error**: Frobenius norm of rotation matrix difference
- **RPY Error**: Euclidean distance of roll-pitch-yaw values

## Usage

### Quick Start

```python
import rclpy
from my_tf_logger.demo_base_log import TFLogger

rclpy.init()
node = TFLogger()

# In another thread or later:
data = node.get_deque_data('camera_to_object1')
latest = data[-1]

print(f"Position: {latest['translation']}")
print(f"Translation Error: {latest['translation_error']}")
```

### Print Summaries

```python
# Print summary of all deques
node.print_deque_summary('camera_to_object1')
node.print_deque_summary('camera_to_object2')
node.print_deque_summary('camera_to_object3')
node.print_deque_summary('camera_to_baselink')
```

### Export Data

```python
import csv
import pickle

# Get deque data
data = node.get_deque_data('camera_to_object1')

# Save to pickle
with open('data.pkl', 'wb') as f:
    pickle.dump(data, f)

# Or export to CSV
with open('data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Timestamp', 'X', 'Y', 'Z', 'R', 'P', 'Y', 'Trans_Error'])
    for entry in data:
        writer.writerow([
            entry['timestamp'],
            entry['translation'][0], entry['translation'][1], entry['translation'][2],
            entry['rotation_rpy'][0], entry['rotation_rpy'][1], entry['rotation_rpy'][2],
            entry['translation_error']
        ])
```

## API Reference

### Methods Added to TFLogger

#### `get_deque_data(deque_name)`
Retrieves all data from a specific deque.

**Parameters:**
- `deque_name` (str): One of 'camera_to_object1', 'camera_to_object2', 'camera_to_object3', 'camera_to_baselink'

**Returns:**
- `list`: List of deque entries, or `None` if invalid name

**Example:**
```python
data = node.get_deque_data('camera_to_object1')
for entry in data:
    print(entry['timestamp'], entry['translation'])
```

#### `print_deque_summary(deque_name)`
Prints a human-readable summary of a deque's contents.

**Parameters:**
- `deque_name` (str): One of the four deque names

**Output:**
```
========== CAMERA_TO_OBJECT1 DEQUE SUMMARY ==========
Total entries: 543
Latest timestamp: 2026-03-19 14:30:45
Translation (x,y,z): [0.5, 0.1, 0.3]
Rotation (r,p,y) rad: [0.785, 0.0, 1.571]
Translation Error: 0.002
Rotation Error: 0.001
RPY Error: 0.0015
```

### Error Calculation Functions

#### `calculate_translation_error(prev, curr)`
```python
error = calculate_translation_error(
    [0.5, 0.1, 0.3],    # Previous [x, y, z]
    [0.501, 0.101, 0.3] # Current [x, y, z]
)
# Returns: Euclidean distance = ~0.00141
```

#### `calculate_rotation_error(prev_matrix, curr_matrix)`
```python
error = calculate_rotation_error(
    prev_rotation_matrix,  # Previous 3x3 rotation matrix
    curr_rotation_matrix   # Current 3x3 rotation matrix
)
# Returns: Frobenius norm of difference
```

#### `calculate_rpy_error(prev_rpy, curr_rpy)`
```python
error = calculate_rpy_error(
    [0.785, 0.0, 1.571],   # Previous [r, p, y]
    [0.785, 0.01, 1.571]   # Current [r, p, y]
)
# Returns: Euclidean distance = ~0.01
```

## Common Use Cases

### 📊 Real-time Monitoring

```python
# Monitor transformation updates
while True:
    latest_obj1 = node.deque_camera_to_object1[-1] if node.deque_camera_to_object1 else None
    if latest_obj1 and latest_obj1['translation_error'] and latest_obj1['translation_error'] > 0.05:
        print("WARNING: Large jump detected!")
    time.sleep(0.1)
```

### 📈 Statistical Analysis

```python
import numpy as np

data = node.get_deque_data('camera_to_object1')
errors = [e['translation_error'] for e in data if e['translation_error']]

print(f"Mean Error: {np.mean(errors):.6f}")
print(f"Std Dev: {np.std(errors):.6f}")
print(f"Max Error: {np.max(errors):.6f}")
```

### 🎯 Trajectory Reconstruction

```python
positions = np.array([entry['translation'] for entry in data])
# Now you have the complete trajectory
print(f"Start: {positions[0]}")
print(f"End: {positions[-1]}")
print(f"Distance traveled: {np.linalg.norm(positions[-1] - positions[0]):.3f}m")
```

### 📁 Data Persistence

```python
import json

# Save before shutdown
all_data = {
    'object1': list(node.deque_camera_to_object1),
    'object2': list(node.deque_camera_to_object2),
    'object3': list(node.deque_camera_to_object3),
    'baselink': list(node.deque_camera_to_baselink),
}

# Convert numpy arrays
for key in all_data:
    for entry in all_data[key]:
        entry['transform_matrix'] = entry['transform_matrix'].tolist()

with open('tf_data.json', 'w') as f:
    json.dump(all_data, f, indent=2)
```

## Performance Characteristics

| Aspect | Value |
|--------|-------|
| **Max entries per deque** | 1000 |
| **Memory per entry** | ~2 KB (4x4 matrix + vectors + floats) |
| **Max memory per deque** | ~2 MB |
| **Total max memory** | ~8 MB (all 4 deques) |
| **Access time** | O(1) - instant access to any entry |
| **Update time** | O(1) - append operation |
| **Search time** | O(n) - linear scan if needed |

## Important Notes

⚠️ **Deque Data is Volatile**
- Data is stored in RAM only
- Data is cleared when the node restarts
- For persistent storage, export to file/pickle periodically

📝 **First Reading**
- The first reading in each deque will have `None` for all error values
- Error values only appear from the second reading onwards

🔄 **Circular Buffer Behavior**
- Once a deque reaches 1000 entries, old entries are automatically removed
- This prevents unlimited memory growth
- Configurable via the `maxlen` parameter in `__init__`

## Troubleshooting

### Q: Data appears empty
**A:** Ensure the node is running and sufficient time has passed for data collection.

### Q: Error values are None
**A:** This is normal for the first reading in each deque. The second reading onwards will have error values.

### Q: Deque is always 1000 entries
**A:** This is expected behavior. The `maxlen=1000` parameter limits storage to 1000 most recent entries.

### Q: How do I persist data?
**A:** Use the provided export functions (pickle, CSV, JSON) or implement your own persistence layer.

## Migration Notes

If you had code using the old file-based approach:

**Old:**
```python
with open('data.txt', 'a') as f:
    f.write(f"Data: {value}")
```

**New:**
```python
data = node.get_deque_data('camera_to_object1')
latest = data[-1]
value = latest['translation']
```

## Additional Resources

- **Implementation Details**: See [DEQUE_IMPLEMENTATION_SUMMARY.md](DEQUE_IMPLEMENTATION_SUMMARY.md)
- **Usage Examples**: See [DEQUE_USAGE_EXAMPLES.md](DEQUE_USAGE_EXAMPLES.md)
- **Export Tool**: Use `deque_exporter.py` for automated data export

## Support

For issues or questions:
1. Check the usage examples in [DEQUE_USAGE_EXAMPLES.md](DEQUE_USAGE_EXAMPLES.md)
2. Review the implementation details in [DEQUE_IMPLEMENTATION_SUMMARY.md](DEQUE_IMPLEMENTATION_SUMMARY.md)
3. Examine the modified [demo_base_log.py](my_tf_logger/demo_base_log.py)

## Summary of Changes

✅ Replaced 4 text file operations with 4 deque operations
✅ Added automatic error calculation between consecutive readings
✅ Implemented helper methods for data access
✅ Created export utility for various formats
✅ Maintained all existing TF broadcasting functionality
✅ No breaking changes to node parameters or ROS2 interface
