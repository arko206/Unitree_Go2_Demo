# Deque Implementation - Usage Examples

## Quick Reference

### 1. **Accessing Data from TFLogger Node**

```python
import rclpy
from my_tf_logger.demo_base_log import TFLogger

rclpy.init()
node = TFLogger()

# In a separate thread or after some data collection:
# Get all data from camera → object1
data = node.get_deque_data('camera_to_object1')

# Access latest entry
if data:
    latest = data[-1]
    print(f"Timestamp: {latest['timestamp']}")
    print(f"Translation: {latest['translation']}")
    print(f"Rotation (r,p,y): {latest['rotation_rpy']}")
    print(f"Transform Matrix:\n{latest['transform_matrix']}")
    print(f"Translation Error from last reading: {latest['translation_error']}")
```

### 2. **Accessing All Four Transformation Streams**

```python
# Get all four transformation streams
camera_to_obj1 = node.get_deque_data('camera_to_object1')
camera_to_obj2 = node.get_deque_data('camera_to_object2')
camera_to_obj3 = node.get_deque_data('camera_to_object3')
camera_to_base = node.get_deque_data('camera_to_baselink')

# Process them
for i, (obj1, obj2, obj3, base) in enumerate(zip(camera_to_obj1, camera_to_obj2, camera_to_obj3, camera_to_base)):
    print(f"\nReading {i}:")
    print(f"  Obj1 Translation: {obj1['translation']}")
    print(f"  Obj2 Translation: {obj2['translation']}")
    print(f"  Obj3 Translation: {obj3['translation']}")
    print(f"  Base Translation: {base['translation']}")
    print(f"  Obj1 Error: {obj1['translation_error']}")
```

### 3. **Analyzing Error Metrics**

```python
import numpy as np

data = node.get_deque_data('camera_to_object1')

# Extract errors (skip first None entry)
trans_errors = [e['translation_error'] for e in data if e['translation_error'] is not None]
rot_errors = [e['rotation_error'] for e in data if e['rotation_error'] is not None]
rpy_errors = [e['rpy_error'] for e in data if e['rpy_error'] is not None]

# Calculate statistics
if trans_errors:
    print(f"Translation Error - Mean: {np.mean(trans_errors):.6f}, Std: {np.std(trans_errors):.6f}")
    print(f"Translation Error - Min: {np.min(trans_errors):.6f}, Max: {np.max(trans_errors):.6f}")

if rot_errors:
    print(f"Rotation Error - Mean: {np.mean(rot_errors):.6f}, Std: {np.std(rot_errors):.6f}")

if rpy_errors:
    print(f"RPY Error - Mean: {np.mean(rpy_errors):.6f}, Std: {np.std(rpy_errors):.6f}")
```

### 4. **Exporting to CSV**

```python
import csv

data = node.get_deque_data('camera_to_object1')

with open('camera_to_object1.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Timestamp', 'Trans_X', 'Trans_Y', 'Trans_Z', 
                     'Roll', 'Pitch', 'Yaw', 'Trans_Error', 'Rot_Error', 'RPY_Error'])
    
    for entry in data:
        writer.writerow([
            entry['timestamp'],
            entry['translation'][0],
            entry['translation'][1],
            entry['translation'][2],
            entry['rotation_rpy'][0],
            entry['rotation_rpy'][1],
            entry['rotation_rpy'][2],
            entry['translation_error'],
            entry['rotation_error'],
            entry['rpy_error']
        ])
```

### 5. **Saving to Pickle**

```python
import pickle

# Save all deques
deques_data = {
    'camera_to_object1': list(node.deque_camera_to_object1),
    'camera_to_object2': list(node.deque_camera_to_object2),
    'camera_to_object3': list(node.deque_camera_to_object3),
    'camera_to_baselink': list(node.deque_camera_to_baselink)
}

with open('tf_logger_data.pkl', 'wb') as f:
    pickle.dump(deques_data, f)

# Load later
with open('tf_logger_data.pkl', 'rb') as f:
    loaded_data = pickle.load(f)
    obj1_data = loaded_data['camera_to_object1']
```

### 6. **Comparing Transformations**

```python
# Compare object1 and object2 transformations
obj1_data = node.get_deque_data('camera_to_object1')
obj2_data = node.get_deque_data('camera_to_object2')

for entry1, entry2 in zip(obj1_data, obj2_data):
    distance = np.linalg.norm(
        np.array(entry1['translation']) - np.array(entry2['translation'])
    )
    print(f"Distance between Obj1 and Obj2: {distance:.6f}m")
```

### 7. **Getting Latest Reading Only**

```python
# Get the most recent reading from each transformation
latest_obj1 = node.deque_camera_to_object1[-1] if node.deque_camera_to_object1 else None
latest_obj2 = node.deque_camera_to_object2[-1] if node.deque_camera_to_object2 else None
latest_obj3 = node.deque_camera_to_object3[-1] if node.deque_camera_to_object3 else None
latest_base = node.deque_camera_to_baselink[-1] if node.deque_camera_to_baselink else None

if latest_obj1:
    print(f"Latest Obj1 Position: {latest_obj1['translation']}")
    print(f"Latest Obj1 Orientation (r,p,y): {latest_obj1['rotation_rpy']}")
```

### 8. **Detecting Large Jumps in Data**

```python
data = node.get_deque_data('camera_to_object1')

# Find entries with large translation errors (potential jumps)
threshold = 0.1  # 10cm
large_jumps = [
    (i, entry) for i, entry in enumerate(data) 
    if entry['translation_error'] is not None and entry['translation_error'] > threshold
]

print(f"Large jumps (> {threshold}m): {len(large_jumps)}")
for i, entry in large_jumps:
    print(f"  Index {i}: {entry['translation_error']:.6f}m at {entry['timestamp']}")
```

### 9. **Print Summary of All Deques**

```python
node.print_deque_summary('camera_to_object1')
node.print_deque_summary('camera_to_object2')
node.print_deque_summary('camera_to_object3')
node.print_deque_summary('camera_to_baselink')
```

### 10. **Time-based Analysis**

```python
from datetime import datetime

data = node.get_deque_data('camera_to_object1')

# Get time range
if data:
    start_time_str = data[0]['timestamp']
    end_time_str = data[-1]['timestamp']
    
    print(f"Data collection start: {start_time_str}")
    print(f"Data collection end: {end_time_str}")
    print(f"Total entries: {len(data)}")
    
    # Calculate update rate
    if len(data) > 1:
        # This is approximate since we only have timestamp strings
        print(f"Expected update rate: ~{10.0} Hz")
```

## Data Structure Reference

Each deque entry is a dictionary with:

```python
{
    'timestamp': '2026-03-19 14:30:45',          # str, ISO timestamp
    'transform_matrix': numpy.ndarray(4, 4),     # 4x4 homogeneous transformation matrix
    'translation': [0.5, 0.1, 0.3],             # list, [x, y, z] in meters
    'rotation_rpy': [0.785, 0.0, 1.571],        # list, [roll, pitch, yaw] in radians
    'translation_error': 0.002,                 # float or None, Euclidean distance error
    'rotation_error': 0.001,                    # float or None, Frobenius norm error
    'rpy_error': 0.0015                         # float or None, RPY Euclidean error
}
```

## Common Patterns

### Pattern 1: Real-time Monitoring
```python
# Check latest data periodically
def check_latest():
    for deque_name in ['camera_to_object1', 'camera_to_object2', 'camera_to_object3', 'camera_to_baselink']:
        data = node.get_deque_data(deque_name)
        if data:
            latest = data[-1]
            if latest['translation_error'] and latest['translation_error'] > 0.05:
                print(f"WARNING: Large jump in {deque_name}")
```

### Pattern 2: Trajectory Analysis
```python
# Analyze trajectory of an object
data = node.get_deque_data('camera_to_object1')
positions = [entry['translation'] for entry in data]
positions = np.array(positions)

# Calculate velocity (approximate)
velocities = np.diff(positions, axis=0) * 10  # Assuming 10Hz
speeds = np.linalg.norm(velocities, axis=1)

print(f"Average speed: {np.mean(speeds):.3f} m/s")
print(f"Max speed: {np.max(speeds):.3f} m/s")
```

### Pattern 3: Data Persistence
```python
# Save data when node is shutting down
def save_before_exit():
    import json
    data_dict = {
        'camera_to_object1': [dict_entry for dict_entry in node.deque_camera_to_object1],
        'camera_to_object2': [dict_entry for dict_entry in node.deque_camera_to_object2],
        'camera_to_object3': [dict_entry for dict_entry in node.deque_camera_to_object3],
        'camera_to_baselink': [dict_entry for dict_entry in node.deque_camera_to_baselink],
    }
    # Convert numpy arrays to lists for JSON serialization
    for key in data_dict:
        for entry in data_dict[key]:
            entry['transform_matrix'] = entry['transform_matrix'].tolist()
            entry['rotation_rpy'] = entry['rotation_rpy'].tolist()
    
    with open('tf_data_backup.json', 'w') as f:
        json.dump(data_dict, f, indent=2)
```
