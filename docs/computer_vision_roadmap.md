# Robotics Computer Vision and AprilTag Localization Roadmap

A practical, step-by-step roadmap from pure OpenCV basics to 6-DOF AprilTag pose estimation and autonomous visual docking for Gizmo AMR.

```
[Phase 0: Pure Python & OpenCV Foundations (Standalone, No ROS)]
  • NumPy uint8 memory layout: shapes (H, W, C) vs (H, W)
  • Coordinate indexing: array[y, x] vs Cartesian (x, y)
  • BGR channel ordering and color conversion
  • Slicing and Regions of Interest (ROI)
  • Drawing primitives: lines, rectangles, circles, HUD text
  • Direct hardware video capture loop (cv2.VideoCapture)
       │
       ▼
[Phase 1: Digital Image Representation & The ROS 2 Vision Pipeline]
  • sensor_msgs/msg/Image format and serialization
  • cv_bridge memory conversion (imgmsg_to_cv2, cv2_to_imgmsg)
  • Thread allocation clamp (OpenMP/OpenBLAS) to eliminate busy-waiting
       │
       ▼
[Phase 2: Spatial Filtering, Segmentation & Contours]
  • 2D convolution kernels and Gaussian blurring
  • Spatial intensity gradients (Sobel) and Canny edge detection
  • Illumination-invariant color masking (HSV color space)
  • Morphological operations (erosion, dilation, opening)
  • Topological contours (findContours) and spatial moments (centroid calculation)
       │
       ▼
[Phase 3: Projective Geometry & The Pinhole Camera Model]
  • Pinhole perspective equations (3D camera points to 2D image coordinates)
  • Camera Matrix K: focal lengths (fx, fy) and principal center (cx, cy)
  • Lens distortion D: Brown-Conrady radial and tangential distortion modeling
  • ROS REP 103 optical conventions (+Z forward, +X right, +Y down)
       │
       ▼
[Phase 4: 2D Fiducial Marker Detection & Homography]
  • AprilTag mechanics: quiet zones, black border, Hamming distance error correction
  • Quad detection pipeline: gradients -> line fitting -> convex quads
  • Planar homography matrix H via Direct Linear Transform (DLT)
       │
       ▼
[Phase 5: 3D Pose Estimation via Perspective-n-Point (PnP)]
  • 3D-to-2D correspondence and the planar fold ambiguity
  • Globally optimal non-linear SQPNP solver vs iterative LM
  • Rodrigues axis-angle representation and unit quaternion conversion
  • TF2 dynamic transform broadcasting (camera_optical_frame -> tag36h11:<id>)
       │
       ▼
[Phase 6: Closed-Loop Visual Servoing & Nav2 Docking]
  • Global map frame transformation chains
  • Proportional velocity controller for visual servoing
  • Nav2 Simple Commander visual waypoint goal generation
```

---
