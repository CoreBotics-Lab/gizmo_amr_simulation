# What I Learned in this Workspace: Systems & Robotics Engineering Notes

A technical reference of core systems engineering, middleware lifecycles, and control paradigms mastered while developing the Gizmo AMR simulation pipeline.

---

## 1. Graceful Teardown and Signal Delegation (`SignalHandlerOptions.NO`)

### The Core Problem: POSIX Signals vs. ROS 2 C-Context Lifecycle
When a user presses `Ctrl+C` in the terminal, the Linux OS kernel broadcasts a `SIGINT` (Signal 2) to the running process group. 

In standard ROS 2 Python nodes:
```python
rclpy.init(args=args)  # Default: SignalHandlerOptions.ALL
```
* ROS 2 installs a C-level POSIX signal handler at the `rcl` / `rmw` middleware layer.
* The moment `SIGINT` is received, the C runtime immediately executes `rcl_shutdown()`.
* This action invalidates the underlying `rcl_context_t` struct and tears down the Data Distribution Service (DDS) participant endpoints in memory.
* When the Python runtime unwinds the stack into `except KeyboardInterrupt:` and reaches the `finally:` block, any attempt to publish a final message fails with:
  ```text
  rclpy._rclpy_pybind11.RCLError: Failed to publish: publisher's context is invalid
  ```

### The Solution: Delegating Signal Handling to Python
By configuring initialization with `SignalHandlerOptions.NO`:
```python
from rclpy.signals import SignalHandlerOptions

rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
```
* ROS 2 is instructed **not** to intercept or hook the OS POSIX signal at the C layer.
* The Linux kernel delivers `SIGINT` directly to the Python interpreter, which raises a standard `KeyboardInterrupt` exception.
* Crucially, the **`rcl_context_t` and DDS participant remain fully valid and open**, allowing publisher objects to transmit network packets inside the `finally:` block.

---

## 2. When and Why This is Required: Actuated Drive Nodes (`/cmd_vel`)

### The Locomotion Safety Hazard
Mobile ground robots (such as two-wheel differential drive or omnidirectional AMRs) operate via continuous velocity integration. Actuators and motor drivers continuously execute the last received setpoint until instructed otherwise or until a safety watchdog expires:

$$\mathbf{u} = \begin{bmatrix} v_x \\ \omega_z \end{bmatrix}$$

* If a line-following or path-tracking node is commanded at $v_x = 0.50\text{ m/s}$ and the process is killed abruptly, the robot will continue driving blindly across the floor at $0.50\text{ m/s}$ until physical impact or watchdog timeout.
* To guarantee deterministic, safe behavior, an autonomous driving node must actively transmit an explicit zero-velocity halt packet (`Twist()` with $v_x = 0.0, \omega_z = 0.0$) during termination.
* Because active network I/O is strictly required inside the exit handler, **actuated nodes must use `SignalHandlerOptions.NO`**.

---

## 3. When This is NOT Required: Passive / Perception Nodes

Nodes that do not command physical hardware or broadcast active shutdown commands do **not** need `SignalHandlerOptions.NO`. Examples include:
* Vision nodes converting image streams (`/camera/image_raw` $\rightarrow$ contours / centroids)
* Sensor filters, state estimators, and telemetry visualizers
* Pure subscriber or monitoring nodes

### Why Passive Nodes Don't Need It
In a perception or passive node, cleanup inside `finally:` consists exclusively of local process operations:
```python
finally:
    cv2.destroyAllWindows()     # Closes local OpenCV HighGUI windows
    node.destroy_node()          # Frees local Python memory
    executor.shutdown()          # Halts local thread pool
    if rclpy.ok():
        rclpy.shutdown()
```
None of these cleanup operations require transmitting packets across the DDS network bus. Even if the C-level signal handler has already invalidated the `rcl_context_t`, destroying local GUI windows and freeing process heap memory succeeds without issue. Using default signal handling in passive nodes is standard and recommended.

---

## 4. Network Transport Realities: The Multi-Packet Stop Burst

Simply calling `publish(Twist())` once inside `finally:` is insufficient in production robotics due to three networking factors:

```python
def stop(self):
    # Broadcast zero velocity and give DDS a moment to flush across the network
    stop_cmd = Twist()
    for _ in range(5):
        self.cmd_vel_pub.publish(stop_cmd)
        time.sleep(0.02)
```

### Why a Single Packet Fails
1. **Asynchronous Non-Blocking Queues**: In ROS 2, `publish()` does not block until the receiver receives the message; it merely writes the serialized packet to an internal DDS participant buffer. If `destroy_node()` is called microseconds later, the socket is destroyed before the background I/O thread flushes the buffer to the OS network stack.
2. **Best-Effort UDP QoS**: Actuation commands on `/cmd_vel` utilize `Best Effort` reliability (unacknowledged UDP packets to avoid latency spikes). If a single UDP packet is dropped or truncated during socket teardown, the robot never receives the halt command.
3. **Controller Sampling Frequencies**: Hardware motor controllers and Gazebo simulation plugins poll setpoints discretely at rates between 20 Hz and 50 Hz ($\Delta t = 20\text{ to }50\text{ ms}$). Spanning 5 packets across $100\text{ ms}$ ($5 \times 0.02\text{ s}$) guarantees that at least one halt frame is received within the controller's active update window.

---

## 5. Developer Responsibility with `SignalHandlerOptions.NO`

Disabling automatic C-level signal handling transfers full lifecycle responsibility to the Python script.

* **Mandatory Manual Teardown**: Because the C runtime will not terminate DDS automatically upon `SIGINT`, you **must** explicitly invoke `rclpy.shutdown()` inside the `finally:` block:
  ```python
  finally:
      controller_node.stop()          # Transmit 5 zero-velocity packets
      controller_node.destroy_node()  # Tear down publisher endpoints
      executor.shutdown()             # Terminate executor threads
      if rclpy.ok():
          rclpy.shutdown()            # Terminate DDS participant context
  ```
* **Consequence of Omission**: If `rclpy.shutdown()` is omitted, the DDS background threads and participant sockets remain open, resulting in orphan ("ghost") background processes that hold port allocations and consume CPU resources.
