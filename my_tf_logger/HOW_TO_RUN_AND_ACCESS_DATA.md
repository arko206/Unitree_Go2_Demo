# ✅ How to Run TFLogger and Access Deque Data

## The Problem You're Having

You're running:
- `demo_base_log.py` in Terminal 1 (collects data in deques)
- `reading_base_log_data.py` in Terminal 2 (tries to read data)

But **they are separate independent nodes**, so the reading script can't access the deques from the collector!

---

## ✅ The Solution: Two Approaches

### **Approach 1: Run Demo and Access Deque Data (RECOMMENDED)**

This is the **simplest way**:

#### **Terminal 1** - Start the TFLogger collector node:
```bash
ros2 run my_tf_logger demo_base_log --ros-args \
  -p parent_frame:=camera \
  -p child_frame:=object_1 \
  -p intermediate_frame:=object_2 \
  -p goal_frame:=object_3 \
  -p rate_hz:=10.0 \
  -p bridge_pose_file:=/home/arka/unitree_sdk2/live_bridge/draft_demo_robot_pose.txt
```

#### **Terminal 2** - Run the data accessor:
```bash
python3 /home/arka/Desktop/ros2_ws/src/my_tf_logger/my_tf_logger/access_deque_data.py
```

This will:
- ✅ Collect data for 10 seconds
- ✅ Print summaries of all deques
- ✅ Show latest readings
- ✅ Analyze errors
- ✅ Export to CSV and Pickle files

---

### **Approach 2: Combined Data Collection + Access**

If you want to collect AND access in one go:

#### **Single Terminal**:
```bash
python3 /home/arka/Desktop/ros2_ws/src/my_tf_logger/my_tf_logger/access_deque_data.py
```

This runs TFLogger internally and automatically accesses the deques.

---

## 🎯 What Each Script Does

### `demo_base_log.py` (Data Collector)
- ✅ Listens to TF frames
- ✅ Collects transformations
- ✅ Stores in deques (in memory)
- ✅ Broadcasts TF frames
- ✅ Runs continuously until you press Ctrl+C

### `access_deque_data.py` (Data Accessor) 
- ✅ Collects data for a set duration (default 10 seconds)
- ✅ Prints deque summaries
- ✅ Shows latest readings
- ✅ Analyzes error statistics
- ✅ Exports to CSV files
- ✅ Exports to Pickle files

---

## 📊 Example Output

When you run `access_deque_data.py`, you'll see:

```
======================================================================
🚀 DEQUE DATA ACCESS SCRIPT
======================================================================

This script collects and accesses deque data from TFLogger

⏱️  Collecting data for 10 seconds...

✅ Data collection complete!

======================================================================
📊 DEQUE SUMMARIES
======================================================================

========== CAMERA_TO_OBJECT1 DEQUE SUMMARY ==========
Total entries: 100
Latest timestamp: 2026-03-19 14:30:45
Translation (x,y,z): [0.5, 0.1, 0.3]
Rotation (r,p,y) rad: [0.785, 0.0, 1.571]
Translation Error: 0.002
Rotation Error: 0.001
RPY Error: 0.0015

======================================================================
📍 LATEST READINGS
======================================================================

🎯 Object-1 (Tag-T1)
   Timestamp: 2026-03-19 14:30:45.123
   Position [x,y,z]: [0.5, 0.1, 0.3]
   Rotation [r,p,y]: [0.785, 0.0, 1.571]
   Translation Error: 0.002000
   Rotation Error: 0.001000
   RPY Error: 0.001500

======================================================================
📈 ERROR STATISTICS
======================================================================

📍 camera_to_object1
   Translation Error:
      Mean: 0.002345 m
      Std:  0.001234 m
      Min:  0.000001 m
      Max:  0.008923 m
   Rotation Error:
      Mean: 0.000987
      Std:  0.000456
      Min:  0.000001
      Max:  0.003456
   RPY Error:
      Mean: 0.001234 rad
      Std:  0.000654 rad
      Min:  0.000001 rad
      Max:  0.004567 rad

======================================================================
💾 EXPORTING TO CSV FILES
======================================================================

✅ Exported 100 entries to: ./camera_to_object1_data.csv
✅ Exported 100 entries to: ./camera_to_object2_data.csv
✅ Exported 100 entries to: ./camera_to_object3_data.csv
✅ Exported 100 entries to: ./camera_to_baselink_data.csv

======================================================================
✅ COMPLETE!
======================================================================
```

---

## 💾 Exported Files

The script automatically creates CSV files in your current directory:
- `camera_to_object1_data.csv`
- `camera_to_object2_data.csv`
- `camera_to_object3_data.csv`
- `camera_to_baselink_data.csv`

You can open these in Excel or any spreadsheet application!

---

## 🔧 Customizing Collection Duration

Edit `access_deque_data.py` and change this line:

```python
accessor.start_collection(duration_seconds=10)  # Change 10 to your desired seconds
```

---

## ⚠️ Common Issues & Solutions

### Issue: "camera_frame" not available
**Solution**: Make sure your TF frames are being published! Check with:
```bash
ros2 topic list
ros2 run tf2_tools view_frames
```

### Issue: No data in deques
**Solution**: 
1. Make sure `demo_base_log.py` is running in Terminal 1
2. Wait at least 10 seconds for data to accumulate
3. Check that TF frames are published

### Issue: Permission denied
**Solution**: Make the script executable:
```bash
chmod +x /home/arka/Desktop/ros2_ws/src/my_tf_logger/my_tf_logger/access_deque_data.py
```

---

## 📝 Running from Your Original Command

Your original command still works to **collect data**:

```bash
ros2 run my_tf_logger demo_base_log --ros-args \
  -p parent_frame:=camera \
  -p child_frame:=object_1 \
  -p intermediate_frame:=object_2 \
  -p goal_frame:=object_3 \
  -p rate_hz:=10.0 \
  -p output_file:=/home/arka/Desktop/Go2_movement_collection/Walk/Left_Walk/Left_Vx_Vyaw/left_x_yaw_d_20.txt \
  -p object_one_to_camera_frame:=/home/arka/Demo_Movement/Debugging_Storage_Files/obj_to_Cam_50.txt \
  -p object_one_to_robot_camera:=/home/arka/Demo_Movement/Debugging_Storage_Files/obj_to_robocam_50.txt \
  -p object_one_to_robot_base:=/home/arka/Demo_Movement/Debugging_Storage_Files/obj_to_base_50.txt \
  -p bridge_pose_file:=/home/arka/unitree_sdk2/live_bridge/draft_demo_robot_pose.txt
```

**But instead of reading text files**, use `access_deque_data.py` to access the data!

---

## Summary

| Task | Command |
|------|---------|
| **Collect data** | `ros2 run my_tf_logger demo_base_log --ros-args ...` |
| **Access & export data** | `python3 access_deque_data.py` |
| **View CSV files** | Open `.csv` files in Excel/spreadsheet |
| **Analyze in Python** | Use pickle files with `pickle.load()` |

**✅ Done! No more text file issues!**
