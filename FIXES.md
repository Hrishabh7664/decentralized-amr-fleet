# Fleet Coordination & Autonomous AMR System Bug Fixes (FIXES.md)

This document tracks all diagnosed failures across the decentralized AMR fleet repository, their reproduction traces, the exact diffs applied, test execution evidence, and topic/interface alignment details.

---

## A1: `state_publisher.py` Battery Topic Alignment & `BatteryState` Support

### 1. Original Bug
`state_publisher` failed when receiving standard `sensor_msgs/msg/BatteryState` messages on the battery topic with an `AttributeError: 'BatteryState' object has no attribute 'data'`. As a result, incoming battery updates were rejected and `/fleet/{robot_id}/state` remained frozen at `100.0%`.

### 2. Topic Alignment
- **Publisher (`battery_monitor.py:115`)**:
  ```python
  self.battery_pub = self.node.create_publisher(Float32, f'/fleet/{self.robot_id}/battery', 10)
  ```
- **Subscriber (`state_publisher.py:49-50`)**:
  ```python
  self.node.create_subscription(Float32, f'/fleet/{self.robot_id}/battery', self.battery_callback, 10)
  self.node.create_subscription(Float32, f'/{self.robot_id}/battery', self.battery_callback, 10)
  ```
  *Both nodes communicate over `/fleet/{robot_id}/battery`, with legacy fallback on `/{robot_id}/battery`.*

### 3. Failing Test Output
```
================================== FAILURES ===================================
_________ TestSectionA.test_a1_state_publisher_receives_battery_state _________

self = <test_section_a.TestSectionA testMethod=test_a1_state_publisher_receives_battery_state>
...
>       self.bus.dispatch("/fleet/robot_1/battery", battery_msg)

tests\mock_ros.py:89: in dispatch
    cb(msg)
src\amr_fleet\amr_fleet\perception\state_publisher.py:80: in battery_callback
    self.battery_pct = msg.data
E   AttributeError: 'BatteryState' object has no attribute 'data'
=========================== short test summary info ===========================
FAILED tests/test_section_a.py::TestSectionA::test_a1_state_publisher_receives_battery_state
============================== 1 failed in 0.20s ==============================
```

### 4. Fix Applied
```diff
--- a/src/amr_fleet/amr_fleet/perception/state_publisher.py
+++ b/src/amr_fleet/amr_fleet/perception/state_publisher.py
@@ -49,2 +49,3 @@
         self.node.create_subscription(Float32, f'/fleet/{self.robot_id}/battery', self.battery_callback, 10)
+        self.node.create_subscription(Float32, f'/{self.robot_id}/battery', self.battery_callback, 10)
         self.node.create_subscription(String, f'/fleet/{self.robot_id}/status', self.status_callback, 10)
@@ -79,2 +80,7 @@
     def battery_callback(self, msg: Any) -> None:
-        """Update local battery percentage reading."""
-        self.battery_pct = msg.data
+        """Update local battery percentage reading supporting Float32 and BatteryState."""
+        if hasattr(msg, 'percentage'):
+            pct = float(msg.percentage)
+            self.battery_pct = pct * 100.0 if pct <= 1.0 else pct
+        elif hasattr(msg, 'data'):
+            self.battery_pct = float(msg.data)
```

### 5. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_a.py -k test_a1
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 2 items

tests\test_section_a.py ..                                               [100%]

============================== 2 passed in 0.06s ==============================
```

---

## A2: `state_publisher.py` Charging Intent Subscription & State Mapping

### 1. Original Bug
`state_publisher` subscribed only to `/fleet/{robot_id}/status`, which was never published by any node in the system. When `battery_monitor` dispatched autonomous charging maneuvers via `CHARGING_INTENT`, `state_publisher` never received them, leaving `current_state` unable to transition to `CHARGING` and `is_charging` stuck at `False`.

### 2. Topic Alignment
- **Publisher (`battery_monitor.py:116`)**:
  ```python
  self.intent_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/charging_intent', 10)
  ```
- **Subscriber (`state_publisher.py:52-53`)**:
  ```python
  self.node.create_subscription(String, f'/fleet/{self.robot_id}/charging_intent', self.charging_intent_callback, 10)
  self.node.create_subscription(String, f'/{self.robot_id}/charging_intent', self.charging_intent_callback, 10)
  ```
  *Aligned topic: `/fleet/{robot_id}/charging_intent` (and fallback `/{robot_id}/charging_intent`).*

### 3. Failing Test Output
```
================================== FAILURES ===================================
_________ TestSectionA.test_a2_charging_intent_maps_to_charging_state _________

self = <test_section_a.TestSectionA testMethod=test_a2_charging_intent_maps_to_charging_state>

    def test_a2_charging_intent_maps_to_charging_state(self):
        ...
        intent_msg = String(data="CHARGING_INTENT:robot_1:station_north_west")
        self.bus.dispatch("/fleet/robot_1/charging_intent", intent_msg)

>       self.assertTrue(sp.is_charging, "is_charging was not set to True upon charging intent!")
E       AssertionError: False is not true : is_charging was not set to True upon charging intent!

tests\test_section_a.py:121: AssertionError
=========================== short test summary info ===========================
FAILED tests/test_section_a.py::TestSectionA::test_a2_charging_intent_maps_to_charging_state
======================= 1 failed, 2 deselected in 0.18s =======================
```

### 4. Fix Applied
```diff
--- a/src/amr_fleet/amr_fleet/perception/state_publisher.py
+++ b/src/amr_fleet/amr_fleet/perception/state_publisher.py
@@ -51,2 +51,4 @@
         self.node.create_subscription(String, f'/fleet/{self.robot_id}/status', self.status_callback, 10)
+        self.node.create_subscription(String, f'/fleet/{self.robot_id}/charging_intent', self.charging_intent_callback, 10)
+        self.node.create_subscription(String, f'/{self.robot_id}/charging_intent', self.charging_intent_callback, 10)
@@ -74,3 +76,3 @@
         if abs(self.current_twist.linear.x) > 0.05 or abs(self.current_twist.angular) > 0.05:
-            if self.current_state != "RESOLVING_CONFLICT":
+            if self.current_state not in ("RESOLVING_CONFLICT", "CHARGING"):
                 self.current_state = "NAVIGATING"
@@ -89,2 +91,15 @@
     def status_callback(self, msg: Any) -> None:
         """Update robot state machine status string."""
         self.current_state = msg.data
+
+    def charging_intent_callback(self, msg: Any) -> None:
+        """Process charging intent notifications and update robot state."""
+        try:
+            data = str(msg.data)
+            parts = data.split(':')
+            action = parts[0].upper()
+            if action in ("CHARGING_INTENT", "CHARGING", "CLAIM"):
+                self.is_charging = True
+                self.current_state = "CHARGING"
+            elif action in ("RELEASE", "COMPLETED", "DISCHARGED", "IDLE"):
+                self.is_charging = False
+                if self.current_state == "CHARGING":
+                    self.current_state = "IDLE"
+        except Exception as e:
+            self.node.get_logger().debug(f"Failed to parse charging intent: {e}")
```

### 5. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_a.py -k test_a2
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 3 items / 2 deselected / 1 selected

tests\test_section_a.py .                                                [100%]

======================= 1 passed, 2 deselected in 0.05s =======================
```

