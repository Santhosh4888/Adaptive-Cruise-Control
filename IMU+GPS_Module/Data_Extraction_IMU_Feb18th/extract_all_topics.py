#!/usr/bin/env python3

import rosbag
import csv

bag_path = "feb_18.bag"

topics = {
    "/fixposition/fpa/corrimu": "corrimu.csv",
    "/fixposition/poiimu": "poiimu.csv",
    "/fixposition/fpa/odomenu": "odomenu.csv",
    "/fixposition/fpa/odometry": "odometry.csv",
    "/fixposition/odometry_smooth": "odometry_smooth.csv",
    "/fixposition/odometry_enu": "odometry_enu.csv",
    "/fixposition/odometry_enu_smooth": "odometry_enu_smooth.csv",
}

bag = rosbag.Bag(bag_path)

for topic, filename in topics.items():
    print(f"Extracting {topic} -> {filename}")
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # ================= HEADER SECTION =================

        writer.writerow(["Topic:", topic])

        if topic == "/fixposition/fpa/corrimu":
            writer.writerow([
                "time",
                "ang_vel_x", "ang_vel_y", "ang_vel_z",
                "lin_acc_x", "lin_acc_y", "lin_acc_z"
            ])

        elif topic == "/fixposition/poiimu":
            writer.writerow([
                "time",
                "ang_vel_x", "ang_vel_y", "ang_vel_z",
                "lin_acc_x", "lin_acc_y", "lin_acc_z"
            ])

        elif topic == "/fixposition/fpa/odometry":
            writer.writerow([
                "time",
                "pos_x", "pos_y", "pos_z",
                "qx", "qy", "qz", "qw",
                "vel_x", "vel_y", "vel_z",
                "ang_vel_x", "ang_vel_y", "ang_vel_z",
                "acc_x", "acc_y", "acc_z"
            ])

        elif topic in [
            "/fixposition/odometry_smooth",
            "/fixposition/odometry_enu",
            "/fixposition/odometry_enu_smooth"
        ]:
            writer.writerow([
                "time",
                "pos_x", "pos_y", "pos_z",
                "qx", "qy", "qz", "qw",
                "vel_x", "vel_y", "vel_z",
                "ang_vel_x", "ang_vel_y", "ang_vel_z"
            ])

        elif topic == "/fixposition/fpa/odomenu":
            writer.writerow([
                "time",

                "pos_x", "pos_y", "pos_z",

                "qx", "qy", "qz", "qw",

                "vel_x", "vel_y", "vel_z",

                "ang_vel_x", "ang_vel_y", "ang_vel_z",

                "acc_x", "acc_y", "acc_z",

                "fusion_status",
                "imu_bias_status",
                "gnss1_status",
                "gnss2_status",
                "wheelspeed_status"
            ])


        # ================= DATA SECTION =================

        for _, msg, t in bag.read_messages(topics=[topic]):

            row = [t.to_sec()]

            # corrimu (nested IMU)
            if topic == "/fixposition/fpa/corrimu":
                row.extend([
                    msg.data.angular_velocity.x,
                    msg.data.angular_velocity.y,
                    msg.data.angular_velocity.z,
                    msg.data.linear_acceleration.x,
                    msg.data.linear_acceleration.y,
                    msg.data.linear_acceleration.z
                ])

            # standard IMU
            elif topic == "/fixposition/poiimu":
                row.extend([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])

            # FPA Odometry (custom message)
            elif topic == "/fixposition/fpa/odometry":
                row.extend([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,

                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,

                    msg.velocity.twist.linear.x,
                    msg.velocity.twist.linear.y,
                    msg.velocity.twist.linear.z,

                    msg.velocity.twist.angular.x,
                    msg.velocity.twist.angular.y,
                    msg.velocity.twist.angular.z,

                    msg.acceleration.x,
                    msg.acceleration.y,
                    msg.acceleration.z
                ])

            # Standard nav_msgs/Odometry
            elif topic in [
                "/fixposition/odometry_smooth",
                "/fixposition/odometry_enu",
                "/fixposition/odometry_enu_smooth"
            ]:
                row.extend([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,

                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,

                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,

                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z
                ])

            # FPA odomenu
            elif topic == "/fixposition/fpa/odomenu":

                row.extend([
                    # Position
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,

                    # Orientation
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,

                    # Linear velocity
                    msg.velocity.twist.linear.x,
                    msg.velocity.twist.linear.y,
                    msg.velocity.twist.linear.z,

                    # Angular velocity
                    msg.velocity.twist.angular.x,
                    msg.velocity.twist.angular.y,
                    msg.velocity.twist.angular.z,

                    # Acceleration
                    msg.acceleration.x,
                    msg.acceleration.y,
                    msg.acceleration.z,

                    # Status flags
                    msg.fusion_status,
                    msg.imu_bias_status,
                    msg.gnss1_status,
                    msg.gnss2_status,
                    msg.wheelspeed_status
                ])


            writer.writerow(row)


bag.close()
print("All topics extracted successfully with headers.")
