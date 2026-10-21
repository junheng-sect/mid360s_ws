"""TF frame names. Bags and nodes from before the 10-21 rename publish the same two gravity-bridge edges as
world->camera_init and body->base_link; init_edge() maps them to the current names."""
ODOM,BASE='drone_odom','drone_base'  # FAST-LIO's gravity-aligned odometry origin; drone body, origin = lidar IMU
ODOM_EDGE,MOUNT_EDGE=(ODOM,'camera_init'),('body',BASE)
LEGACY={('world','camera_init'):ODOM_EDGE,('body','base_link'):MOUNT_EDGE}


def init_edge(parent,child):
    """Gravity-bridge static edge (parent, child) under the current names, or None for any other edge."""
    key=parent.strip('/'),child.strip('/');key=LEGACY.get(key,key)
    return key if key in (ODOM_EDGE,MOUNT_EDGE) else None
