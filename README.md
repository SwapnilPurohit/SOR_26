# Session 4 Assignment Report

## Short Report

### 1. How you estimated distance?
The robot subscribes to the `/camera/depth_image` topic from the simulated RGB-D camera. When YOLOv8 detects the target object, it outputs a bounding box. I calculate the exact center coordinate `(cx, cy)` of this box. However, relying on a single pixel from a depth camera can be unreliable due to sensor noise or dead pixels. To solve this, I extract a 5x5 pixel window around the center point. We filter out any invalid or zero-depth values and take the **median** of the remaining pixels. This provides a highly robust and stable distance estimate to the target in meters.

### 2. How your search behaviour works?
When a mission begins, the robot initializes in a `SEARCHING` state. Because the robot's camera has a limited forward-facing field of view (approx. 90 degrees), I implemented a continuous spin behavior. The robot publishes a constant angular velocity (`twist.angular.z = 0.5 rad/s`) while remaining stationary. This simple rotational motion effectively expands the camera's perception into a 360-degree panoramic sweep of the room. It continues scanning until the user-specified target object appears in the YOLOv8 detection list.

### 3. How your robot decides when to move and stop?
Once the target is spotted, the robot transitions to the `TRACKING` state.
*   **Move:** The robot calculates the horizontal offset (error) between the object's bounding box center and the absolute center of the camera frame. A tuned Proportional (P) controller converts this pixel error into an angular steering velocity (`twist.angular.z`), allowing the robot to smoothly turn and keep the object centered while simultaneously driving forward at a constant speed (`twist.linear.x = 0.4 m/s`).
*   **Stop:** During the approach, the robot continuously monitors the estimated distance to the object using the depth camera. Once the distance falls below a safe stopping threshold (`1.0 meter`), the robot immediately publishes zero velocity (`twist.linear.x = 0.0, twist.angular.z = 0.0`), halting safely in front of the target and transitioning to the `COMPLETED` state.
