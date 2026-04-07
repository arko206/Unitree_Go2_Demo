# 🚀 QUICK START GUIDE - 2 Terminal Setup

## Setup Instructions (Copy & Paste)

### Terminal 1: Start Data Collector
```bash
ros2 run my_tf_logger demo_base_log --ros-args \
  -p parent_frame:=camera \
  -p child_frame:=object_1 \
  -p intermediate_frame:=object_2 \
  -p goal_frame:=object_3 \
  -p rate_hz:=10.0 \
  -p bridge_pose_file:=/home/arka/unitree_sdk2/live_bridge/draft_demo_robot_pose.txt
```

**Output should show**:
```
[INFO] Logging TF 'camera_frame' -> 'object' at 10.0 Hz using deques
[INFO] Deques initialized for camera->object1, object2, object3, and base_link
```

Leave this running!

---

### Terminal 2: Access & Export Data
```bash
python3 /home/arka/Desktop/ros2_ws/src/my_tf_logger/my_tf_logger/access_deque_data.py
```

**Output will show**:
- ✅ Deque summaries
- ✅ Latest readings
- ✅ Error statistics
- ✅ CSV/Pickle exports

---

## What You Get

### 📊 On-Screen Output
```
📍 Object-1 (Tag-T1)
   Position [x,y,z]: [0.5, 0.1, 0.3]
   Rotation [r,p,y]: [0.785, 0.0, 1.571]
   Translation Error: 0.002000
```

### 💾 CSV Files (in current directory)
```
camera_to_object1_data.csv
camera_to_object2_data.csv
camera_to_object3_data.csv
camera_to_baselink_data.csv
```

Open in **Excel** or **Google Sheets**!

---

## No More Text Files!

| Old Way | New Way |
|---------|---------|
| Read `.txt` files | Access deques in memory |
| Files corrupt | Data reliable |
| Manual parsing | Structured data |
| Slow disk access | Fast RAM access |

---

## Troubleshooting

**Q: No data showing?**
- A: Make sure Terminal 1 is still running

**Q: "camera_frame not available"?**
- A: Check TF frames are being published

**Q: Want more collection time?**
- A: Edit line in `access_deque_data.py`:
  ```python
  accessor.start_collection(duration_seconds=30)  # Change 10 to 30
  ```

---

**That's it! You're ready to go!** 🎉
