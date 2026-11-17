# YWL source and adaptations

Source: [YWL0720/FAST-LOCALIZATION at 93da4f462b8da16679c18a8aea9545ed4cb3c71b](https://github.com/YWL0720/FAST-LOCALIZATION/tree/93da4f462b8da16679c18a8aea9545ed4cb3c71b).
The included IKFoM and ikd-Tree sources come from the same tree. See LICENSE and the original notices.
The runtime preserves the point-to-plane observation, IEKF and IMU deskew. The ROS wrapper replaces the
ScanContext thread, input map database, keyboard prompt, and global state reset. It adapts the Livox
message to driver2, keeps the calibrated extrinsics fixed, and isolates parameters, topics and artifacts.
