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

---

## A3: `intent_publisher.py` Subscribes to Planned Path Topic

### 1. Original Bug
`intent_publisher` was susceptible to missing path waypoints if published on either `/fleet/{robot_id}/plan` or hardware/legacy topic `/{robot_id}/plan`. If misaligned, `planned_path` remained empty (`[]`), broadcasting an empty route in `/fleet/{robot_id}/intent`.

### 2. Topic Alignment
- **Publisher (`global_planner.py:291`)**:
  ```python
  self.plan_pub = self.node.create_publisher(Path, f'/fleet/{self.robot_id}/plan', 10)
  ```
- **Subscriber (`intent_publisher.py:47-48`)**:
  ```python
  self.node.create_subscription(Path, f'/fleet/{self.robot_id}/plan', self.plan_callback, 10)
  self.node.create_subscription(Path, f'/{self.robot_id}/plan', self.plan_callback, 10)
  ```
  *Aligned topic: `/fleet/{robot_id}/plan` (with dual subscription on `/{robot_id}/plan`).*

### 3. Failing Test Output
```
================================== FAILURES ===================================
__________ TestSectionA.test_a3_intent_publisher_subscribes_to_plan ___________

self = <test_section_a.TestSectionA testMethod=test_a3_intent_publisher_subscribes_to_plan>

    def test_a3_intent_publisher_subscribes_to_plan(self):
        ...
        self.bus.dispatch("/robot_1/plan", path_msg)
>       self.assertEqual(len(ip.planned_path), 3, "planned_path was empty after publishing on /robot_1/plan!")
E       AssertionError: 0 != 3 : planned_path was empty after publishing on /robot_1/plan!

tests\test_section_a.py:154: AssertionError
=========================== short test summary info ===========================
FAILED tests/test_section_a.py::TestSectionA::test_a3_intent_publisher_subscribes_to_plan
======================= 1 failed, 3 deselected in 0.19s =======================
```

### 4. Fix Applied
```diff
--- a/src/amr_fleet/amr_fleet/utils/intent_publisher.py
+++ b/src/amr_fleet/amr_fleet/utils/intent_publisher.py
@@ -47,2 +47,4 @@
         self.node.create_subscription(Path, f'/fleet/{self.robot_id}/plan', self.plan_callback, 10)
+        self.node.create_subscription(Path, f'/{self.robot_id}/plan', self.plan_callback, 10)
         self.node.create_subscription(Point, f'/fleet/{self.robot_id}/goal', self.goal_callback, 10)
+        self.node.create_subscription(Point, f'/{self.robot_id}/goal', self.goal_callback, 10)
@@ -57,2 +59,10 @@
     def plan_callback(self, msg: Any) -> None:
-        """Update cached planned path waypoints."""
-        self.planned_path = [Vector2D(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
+        """Update cached planned path waypoints."""
+        path = []
+        for ps in getattr(msg, 'poses', []):
+            if hasattr(ps, 'pose'):
+                path.append(Vector2D(ps.pose.position.x, ps.pose.position.y))
+            elif hasattr(ps, 'position'):
+                path.append(Vector2D(ps.position.x, ps.position.y))
+            elif hasattr(ps, 'x') and hasattr(ps, 'y'):
+                path.append(Vector2D(ps.x, ps.y))
+        self.planned_path = path
```

### 5. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_a.py -k test_a3
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 4 items / 3 deselected / 1 selected

tests\test_section_a.py .                                                [100%]

======================= 1 passed, 3 deselected in 0.06s =======================
```

---

## C1: `battery_monitor.py` Atomic Charging Station Claims & Contention Arbitration

### 1. Original Bug
When multiple AMRs simultaneously required charging, both selected the nearest dock and published `CLAIM` messages concurrently without checking or handling cross-traffic contention. As a result, both robots committed to the same station dock, racing to the identical charger.

### 2. Protocol & Arbitration
- `CLAIM:{robot_id}:{station_id}:{timestamp}` published over `/fleet/charging_stations`.
- In `station_callback`: When a peer's `CLAIM` matches the station currently claimed by this robot, first-come-wins arbitration evaluates the timestamps (breaking ties deterministically by `robot_id`).
- If this robot loses the contention, it immediately backs off, vacates the dock reservation, re-evaluates the cost function for remaining available stations, and dispatches a new claim for an alternate dock.

### 3. Failing Test Output
```
================================== FAILURES ===================================
____ TestSectionC.test_c1_atomic_station_claims_and_contention_arbitration ____

self = <test_section_c.TestSectionC testMethod=test_c1_atomic_station_claims_and_contention_arbitration>
...
>       self.assertNotEqual(
            bm_2.monitor.assigned_station.station_id, "station_east",
            "Robot 2 failed to back off and re-select upon losing claim contention!"
        )
E       AssertionError: 'station_east' == 'station_east' : Robot 2 failed to back off and re-select upon losing claim contention!

tests\test_section_c.py:124: AssertionError
=========================== short test summary info ===========================
FAILED tests/test_section_c.py::TestSectionC::test_c1_atomic_station_claims_and_contention_arbitration
======================= 1 failed, 2 deselected in 0.18s =======================
```

### 4. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_c.py -k test_c1
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 3 items / 2 deselected / 1 selected

tests\test_section_c.py .                                                [100%]

======================= 1 passed, 2 deselected in 0.06s =======================
```

---

## C2: `battery_monitor.py` Occupancy Lease & Heartbeat Timeout

### 1. Original Bug
If a robot crashed, shut down, or disconnected while occupying or en route to a charging station, the dock remained permanently marked as occupied (`is_occupied=True`), stranding other AMRs and permanently reducing charging capacity.

### 2. Required Fix
- Added `last_heartbeat` tracking to `ChargingStation`.
- Active occupants re-broadcast `HEARTBEAT:{robot_id}:{station_id}:{timestamp}` every 2 seconds on `/fleet/charging_stations`.
- All monitor nodes run `prune_stale_leases(timeout_s=6.0)` on every tick, evicting any station occupied by a silent peer after 6.0 seconds of missed heartbeats.

### 3. Failing Test Output
```
================================== FAILURES ===================================
_________ TestSectionC.test_c2_dock_lease_heartbeat_timeout_on_crash __________

self = <test_section_c.TestSectionC testMethod=test_c2_dock_lease_heartbeat_timeout_on_crash>
...
>       monitor.prune_stale_leases(timeout_s=6.0, current_time=base_time + 4.0)
E       AttributeError: 'BatteryMonitor' object has no attribute 'prune_stale_leases'

tests\test_section_c.py:147: AttributeError
=========================== short test summary info ===========================
FAILED tests/test_section_c.py::TestSectionC::test_c2_dock_lease_heartbeat_timeout_on_crash
======================= 1 failed, 2 deselected in 0.18s =======================
```

### 4. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_c.py -k test_c2
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 3 items / 2 deselected / 1 selected

tests\test_section_c.py .                                                [100%]

======================= 1 passed, 2 deselected in 0.05s =======================
```

---

## C3: `battery_monitor.py` Charging State Machine Hysteresis & Debounce

### 1. Original Bug
The charging state machine stopped charging at 95%, but had no hysteresis lower bound. Any noise or minor discharge that brought the level slightly below 95% (e.g. 94.8% -> 95.1% -> 94.9%) caused rapid fluttering between charging and uncharging on every cycle.

### 2. Required Fix
Introduced hysteresis boundaries:
- `stop_charging_threshold = 95.0%`
- `start_charging_threshold = 90.0%`
Once full charge is achieved (>= 95%), the battery monitor will not re-enter charging mode while hovering above 90.0%, preventing chatter.

### 3. Failing Test Output
```
================================== FAILURES ===================================
____________ TestSectionC.test_c3_charging_hysteresis_and_debounce ____________

self = <test_section_c.TestSectionC testMethod=test_c3_charging_hysteresis_and_debounce>
...
>       self.assertEqual(toggle_count, 0, f"Charging toggled {toggle_count} times around 95%!")
E       AssertionError: 5 != 0 : Charging toggled 5 times around 95%!

tests\test_section_c.py:80: AssertionError
=========================== short test summary info ===========================
FAILED tests/test_section_c.py::TestSectionC::test_c3_charging_hysteresis_and_debounce
============================== 1 failed in 0.17s ==============================
```

### 4. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_c.py -k test_c3
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 1 item

tests\test_section_c.py .                                                [100%]

============================== 1 passed in 0.05s ==============================
```


---

## A4: `intent_publisher.py` Subscribes to Goal Topic

### 1. Original Bug
Navigation goals are dispatched by `fleet_controller` and `task_allocator` on `/fleet/{robot_id}/goal`. If `intent_publisher` only subscribed to `/{robot_id}/goal`, incoming fleet goals were dropped, causing `current_goal` to remain at default `(0, 0)`.

### 2. Topic Alignment
- **Publishers**:
  - `fleet_controller.py:107`: `self.create_publisher(Point, f'/fleet/{r_id}/goal', 10)`
  - `battery_monitor.py:117`: `self.create_publisher(Point, f'/fleet/{self.robot_id}/goal', 10)`
  - `task_allocator.py:115`: `self.create_publisher(Point, f'/fleet/{self.robot_id}/goal', 10)`
- **Subscriber (`intent_publisher.py:49-50`)**:
  ```python
  self.node.create_subscription(Point, f'/fleet/{self.robot_id}/goal', self.goal_callback, 10)
  self.node.create_subscription(Point, f'/{self.robot_id}/goal', self.goal_callback, 10)
  ```
  *Aligned topic: `/fleet/{robot_id}/goal` (with fallback on `/{robot_id}/goal`).*

### 3. Test & Verification
Regression test `test_a4_intent_publisher_subscribes_to_goal` asserts that goals published on both topics correctly update `current_goal` and are reflected in `/fleet/{robot_id}/intent`.

**Passing Test Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 5 items / 4 deselected / 1 selected

tests\test_section_a.py .                                                [100%]

======================= 1 passed, 4 deselected in 0.05s =======================
```

---

## A5: `conflict_resolver.py` Priority and Token Status Publisher Alignment

### 1. Original Bug
`intent_publisher` subscribed to `/fleet/{robot_id}/priority` and `/fleet/{robot_id}/token_status`, but nothing in `conflict_resolver.py` published these values. As a consequence, `intent_publisher`'s priority score was always stuck at `0.5` and `has_token` was always `False`.

### 2. Topic Alignment
- **Publishers (`conflict_resolver.py:216-217`)**:
  ```python
  self.priority_pub = self.node.create_publisher(Float32, f'/fleet/{self.robot_id}/priority', 10)
  self.token_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/token_status', 10)
  ```
  *Published in `resolution_loop` (`Float32(data=my_priority)`) and upon token acquisition/release (`String(data=f"{corridor_id}:HELD")` / `RELEASED`).*
- **Subscribers (`intent_publisher.py:51-52`)**:
  ```python
  self.node.create_subscription(Float32, f'/fleet/{self.robot_id}/priority', self.priority_callback, 10)
  self.node.create_subscription(String, f'/fleet/{self.robot_id}/token_status', self.token_callback, 10)
  ```

### 3. Failing Test Output
```
================================== FAILURES ===================================
_ TestSectionA.test_a5_conflict_resolver_publishes_priority_and_token_status __

self = <test_section_a.TestSectionA testMethod=test_a5_conflict_resolver_publishes_priority_and_token_status>

    def test_a5_conflict_resolver_publishes_priority_and_token_status(self):
        ...
        priority_pubs = [p for p in cr_node.publishers if p.topic == "/fleet/robot_1/priority"]
>       self.assertTrue(len(priority_pubs) > 0, "ConflictResolverNode lacks /fleet/{robot_id}/priority publisher!")
E       AssertionError: False is not true : ConflictResolverNode lacks /fleet/{robot_id}/priority publisher!

tests\test_section_a.py:220: AssertionError
=========================== short test summary info ===========================
FAILED tests/test_section_a.py::TestSectionA::test_a5_conflict_resolver_publishes_priority_and_token_status
======================= 1 failed, 5 deselected in 0.19s =======================
```

### 4. Fix Applied
```diff
--- a/src/amr_fleet/amr_fleet/utils/conflict_resolver.py
+++ b/src/amr_fleet/amr_fleet/utils/conflict_resolver.py
@@ -215,2 +215,4 @@
         self.replan_pub = self.node.create_publisher(Point, f'/fleet/{self.robot_id}/replan_trigger', 10)
+        self.priority_pub = self.node.create_publisher(Float32, f'/fleet/{self.robot_id}/priority', 10)
+        self.token_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/token_status', 10)
@@ -277,2 +279,20 @@
+    def request_corridor_token(self, corridor_id: str) -> bool:
+        """Request corridor token and broadcast status."""
+        granted = self.resolver.request_corridor_token(corridor_id)
+        if granted:
+            msg = String()
+            msg.data = f"{corridor_id}:HELD"
+            self.token_pub.publish(msg)
+        return granted
+
+    def release_corridor_token(self, corridor_id: str) -> None:
+        """Release corridor token and broadcast status."""
+        self.resolver.release_corridor_token(corridor_id)
+        msg = String()
+        msg.data = f"{corridor_id}:RELEASED"
+        self.token_pub.publish(msg)
+
     def resolution_loop(self) -> None:
         ...
         my_priority = self.resolver.compute_composite_priority(dist_to_goal, 0.8, self.battery_pct)
+        p_msg = Float32()
+        p_msg.data = float(my_priority)
+        self.priority_pub.publish(p_msg)
```

### 5. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_a.py -k test_a5
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 6 items / 5 deselected / 1 selected

tests\test_section_a.py .                                                [100%]

======================= 1 passed, 5 deselected in 0.05s =======================
```

---

## B1: `conflict_resolver.py` Nearby Peers Stationary Calculation

### 1. Original Bug
`resolution_loop` in `ConflictResolverNode` called `self.resolver.update_deadlock_state(..., nearby_peers_stationary=True)` unconditionally. Any robot that stopped for > 3.0 seconds (e.g. at an intersection or waiting for clearance) was incorrectly flagged as deadlocked, even when surrounding peers were moving freely.

### 2. Required Fix
Compute `nearby_peers_stationary` dynamically from peer positions and velocities within a radius (default `2.5m`). Deadlock is triggered only when:
1. The robot itself is stopped (speed <= threshold) for > 3.0 seconds with goal unreached.
2. At least one peer AMR is nearby in the collision vicinity.
3. ALL nearby peer AMRs are also stationary (speed <= threshold).

### 3. Failing Test Output
```
================================== FAILURES ===================================
___________ TestSectionB.test_b1_unit_nearby_peers_stationary_logic ___________

self = <test_section_b.TestSectionB testMethod=test_b1_unit_nearby_peers_stationary_logic>

    def test_b1_unit_nearby_peers_stationary_logic(self):
        ...
>       peers_stationary = resolver.compute_nearby_peers_stationary(
                           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            my_pos=Vector2D(0.0, 0.0),
            peer_positions=peer_positions,
            peer_speeds=peer_speeds,
            radius=2.5
        )
E       AttributeError: 'ConflictResolver' object has no attribute 'compute_nearby_peers_stationary'

tests\test_section_b.py:57: AttributeError
=========================== short test summary info ===========================
FAILED tests/test_section_b.py::TestSectionB::test_b1_unit_nearby_peers_stationary_logic
============================== 1 failed in 0.18s ==============================
```

### 4. Fix Applied
```diff
--- a/src/amr_fleet/amr_fleet/utils/conflict_resolver.py
+++ b/src/amr_fleet/amr_fleet/utils/conflict_resolver.py
@@ -131,2 +131,20 @@
+    def compute_nearby_peers_stationary(
+        self,
+        my_pos: Vector2D,
+        peer_positions: Dict[str, Vector2D],
+        peer_speeds: Dict[str, float],
+        radius: float = 2.5,
+        speed_threshold: Optional[float] = None
+    ) -> bool:
+        """Compute whether nearby peers within radius are stationary."""
+        thresh = speed_threshold if speed_threshold is not None else self.speed_threshold
+        nearby_count = 0
+        for peer_id, pos in peer_positions.items():
+            if peer_id == self.robot_id:
+                continue
+            if my_pos.distance_to(pos) <= radius:
+                nearby_count += 1
+                if peer_speeds.get(peer_id, 0.0) > thresh:
+                    return False
+        return nearby_count > 0
@@ -334,3 +352,5 @@
-        is_deadlock = self.resolver.update_deadlock_state(
-            self.current_speed, dist_to_goal, nearby_peers_stationary=True
-        )
+        nearby_stat = self.resolver.compute_nearby_peers_stationary(
+            self.current_pos, self.peer_positions, self.peer_speeds
+        )
+        is_deadlock = self.resolver.update_deadlock_state(
+            self.current_speed, dist_to_goal, nearby_peers_stationary=nearby_stat
+        )
```

### 5. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_b.py -k test_b1
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 2 items

tests\test_section_b.py ..                                               [100%]

============================== 2 passed in 0.05s ==============================
```

---

## B2: `conflict_resolver.py` Corridor Token Contention & Broadcast Protocol

### 1. Original Bug
`request_corridor_token` in `ConflictResolver` only wrote to a local internal dictionary; no `TOKEN_REQ` was ever broadcast to peers over `/fleet/conflicts`. As a result, when two AMRs contended for the same narrow aisle simultaneously, both granted themselves the token locally, leading to head-on deadlock and mutual exclusion violation.

### 2. Protocol & Topic Alignment
- **Broadcast Topics**:
  - `TOKEN_REQ:{robot_id}:{corridor_id}:{priority}` on `/fleet/conflicts`
  - `TOKEN_REL:{robot_id}:{corridor_id}` on `/fleet/conflicts`
  - Status output: `{corridor_id}:HELD` and `{corridor_id}:RELEASED` on `/fleet/{robot_id}/token_status`
- **Contention Arbitration**:
  - Peers track active token holders. If held by another AMR, new requests are rejected (`False`).
  - Upon release, the corridor token is freed across all listening peers.

### 3. Failing Test Output
```
================================== FAILURES ===================================
________ TestSectionB.test_b2_corridor_token_contention_and_broadcast _________

self = <test_section_b.TestSectionB testMethod=test_b2_corridor_token_contention_and_broadcast>
...
>       self.assertTrue(any("TOKEN_REQ:robot_1:corridor_main" in m for m in fleet_conflicts_msgs),
                        "TOKEN_REQ was never broadcast on /fleet/conflicts!")
E       AssertionError: False is not true : TOKEN_REQ was never broadcast on /fleet/conflicts!

tests\test_section_b.py:172: AssertionError
=========================== short test summary info ===========================
FAILED tests/test_section_b.py::TestSectionB::test_b2_corridor_token_contention_and_broadcast
======================= 1 failed, 2 deselected in 0.18s =======================
```

### 4. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_b.py -k test_b2
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 3 items / 2 deselected / 1 selected

tests\test_section_b.py .                                                [100%]

======================= 1 passed, 2 deselected in 0.05s =======================
```

---

## B3: `conflict_resolver.py` Pruning Stale Peers in Token & Deadlock State

### 1. Original Bug
No pruning or timeout existed for stale peer AMR states. If a peer AMR acquired a corridor token and crashed or lost communication, the corridor remained locked permanently. Similarly, stale peer velocity and positions remained indefinitely in memory.

### 2. Required Fix
- Implemented `prune_stale_tokens(timeout_s=10.0)` in `ConflictResolver`: automatically revokes tokens held by silent peers after 10 seconds.
- Implemented `prune_stale_peers(timeout_s=10.0)` in `ConflictResolverNode`: evicts dead peers from position, velocity, and intent registries, and executes periodic pruning on every resolution loop tick.

### 3. Failing Test Output
```
================================== FAILURES ===================================
____ TestSectionB.test_b3_pruning_stale_peers_in_token_and_deadlock_state _____

self = <test_section_b.TestSectionB testMethod=test_b3_pruning_stale_peers_in_token_and_deadlock_state>
...
>       resolver.prune_stale_tokens(timeout_s=10.0, current_time=base_time + 5.0)
E       AttributeError: 'ConflictResolver' object has no attribute 'prune_stale_tokens'

tests\test_section_b.py:201: AttributeError
=========================== short test summary info ===========================
FAILED tests/test_section_b.py::TestSectionB::test_b3_pruning_stale_peers_in_token_and_deadlock_state
======================= 1 failed, 3 deselected in 0.18s =======================
```

### 4. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_b.py -k test_b3
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 4 items / 3 deselected / 1 selected

tests\test_section_b.py .                                                [100%]

======================= 1 passed, 3 deselected in 0.06s =======================
```

---

## C1: `battery_monitor.py` Atomic Charging Station Claims & Contention Arbitration

### 1. Original Bug
When multiple AMRs simultaneously required charging, both selected the nearest dock and published `CLAIM` messages concurrently without checking or handling cross-traffic contention. As a result, both robots committed to the same station dock, racing to the identical charger.

### 2. Protocol & Arbitration
- `CLAIM:{robot_id}:{station_id}:{timestamp}` published over `/fleet/charging_stations`.
- In `station_callback`: When a peer's `CLAIM` matches the station currently claimed by this robot, first-come-wins arbitration evaluates the timestamps (breaking ties deterministically by `robot_id`).
- If this robot loses the contention, it immediately backs off, vacates the dock reservation, re-evaluates the cost function for remaining available stations, and dispatches a new claim for an alternate dock.

### 3. Failing Test Output
```
================================== FAILURES ===================================
____ TestSectionC.test_c1_atomic_station_claims_and_contention_arbitration ____

self = <test_section_c.TestSectionC testMethod=test_c1_atomic_station_claims_and_contention_arbitration>
...
>       self.assertNotEqual(
            bm_2.monitor.assigned_station.station_id, "station_east",
            "Robot 2 failed to back off and re-select upon losing claim contention!"
        )
E       AssertionError: 'station_east' == 'station_east' : Robot 2 failed to back off and re-select upon losing claim contention!

tests\test_section_c.py:124: AssertionError
=========================== short test summary info ===========================
FAILED tests/test_section_c.py::TestSectionC::test_c1_atomic_station_claims_and_contention_arbitration
======================= 1 failed, 2 deselected in 0.18s =======================
```

### 4. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_c.py -k test_c1
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 3 items / 2 deselected / 1 selected

tests\test_section_c.py .                                                [100%]

======================= 1 passed, 2 deselected in 0.06s =======================
```

---

## C2: `battery_monitor.py` Occupancy Lease & Heartbeat Timeout

### 1. Original Bug
If a robot crashed, shut down, or disconnected while occupying or en route to a charging station, the dock remained permanently marked as occupied (`is_occupied=True`), stranding other AMRs and permanently reducing charging capacity.

### 2. Required Fix
- Added `last_heartbeat` tracking to `ChargingStation`.
- Active occupants re-broadcast `HEARTBEAT:{robot_id}:{station_id}:{timestamp}` every 2 seconds on `/fleet/charging_stations`.
- All monitor nodes run `prune_stale_leases(timeout_s=6.0)` on every tick, evicting any station occupied by a silent peer after 6.0 seconds of missed heartbeats.

### 3. Failing Test Output
```
================================== FAILURES ===================================
_________ TestSectionC.test_c2_dock_lease_heartbeat_timeout_on_crash __________

self = <test_section_c.TestSectionC testMethod=test_c2_dock_lease_heartbeat_timeout_on_crash>
...
>       monitor.prune_stale_leases(timeout_s=6.0, current_time=base_time + 4.0)
E       AttributeError: 'BatteryMonitor' object has no attribute 'prune_stale_leases'

tests\test_section_c.py:147: AttributeError
=========================== short test summary info ===========================
FAILED tests/test_section_c.py::TestSectionC::test_c2_dock_lease_heartbeat_timeout_on_crash
======================= 1 failed, 2 deselected in 0.18s =======================
```

### 4. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_c.py -k test_c2
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 3 items / 2 deselected / 1 selected

tests\test_section_c.py .                                                [100%]

======================= 1 passed, 2 deselected in 0.05s =======================
```

---

## C3: `battery_monitor.py` Charging State Machine Hysteresis & Debounce

### 1. Original Bug
The charging state machine stopped charging at 95%, but had no hysteresis lower bound. Any noise or minor discharge that brought the level slightly below 95% (e.g. 94.8% -> 95.1% -> 94.9%) caused rapid fluttering between charging and uncharging on every cycle.

### 2. Required Fix
Introduced hysteresis boundaries:
- `stop_charging_threshold = 95.0%`
- `start_charging_threshold = 90.0%`
Once full charge is achieved (>= 95%), the battery monitor will not re-enter charging mode while hovering above 90.0%, preventing chatter.

### 3. Failing Test Output
```
================================== FAILURES ===================================
____________ TestSectionC.test_c3_charging_hysteresis_and_debounce ____________

self = <test_section_c.TestSectionC testMethod=test_c3_charging_hysteresis_and_debounce>
...
>       self.assertEqual(toggle_count, 0, f"Charging toggled {toggle_count} times around 95%!")
E       AssertionError: 5 != 0 : Charging toggled 5 times around 95%!

tests\test_section_c.py:80: AssertionError
=========================== short test summary info ===========================
FAILED tests/test_section_c.py::TestSectionC::test_c3_charging_hysteresis_and_debounce
============================== 1 failed in 0.17s ==============================
```

### 4. Passing Test Output
**Command:**
```powershell
.\.venv\Scripts\pytest.exe tests/test_section_c.py -k test_c3
```
**Output:**
```
============================= test session starts =============================
platform win32 -- Python 3.12.14, pytest-9.2.0, pluggy-1.7.0
rootdir: C:\Users\hrishabh\githubmy\SWARM
configfile: pyproject.toml
collected 1 item

tests\test_section_c.py .                                                [100%]

============================== 1 passed in 0.05s ==============================
```







