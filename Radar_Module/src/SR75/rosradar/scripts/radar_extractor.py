#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from custom_msgs.msg import RadarDetectionArray,RadarDetection

class RadarLeadExtractor:

    def __init__(self):
        rospy.init_node("radar_lead_extractor")
        rospy.loginfo("Radar extractor node started and waiting for data")


        self.lane_half_width = 1.75
        self.max_range = 50.0
        self.alpha = 0.3

        # self.v_ego = 0.0
        self.v_rel_filt = 0.0  #Filtered relative velocity

        rospy.Subscriber("/radar_detections",
                         RadarDetectionArray,
                         self.radar_callback)

        # rospy.Subscriber("/ego_speed",
        #                  Float32,
        #                  self.ego_speed_callback)

        self.pub_lead_dist = rospy.Publisher("/lead_distance",
                                             Float32,
                                             queue_size=10)

        self.pub_lead_rel_vel = rospy.Publisher("/lead_relative_velocity",
                                              Float32,
                                              queue_size=10)

        rospy.loginfo("Radar Lead Extractor running")

    # def ego_speed_callback(self, msg):
    #     self.v_ego = msg.data

    def radar_callback(self, msg):
        rospy.logdebug("Radar callback triggered")

        min_x = float("inf")
        best_det = None

        for det in msg.detections:

            x = det.position.x
            y = det.position.y

            if x <= 0.0:
                continue

            if abs(y) > self.lane_half_width:
                continue

            if det.range > self.max_range:
                continue

            if x < min_x:
                min_x = x
                best_det = det

        if best_det is None:
            return

        d_lead = best_det.position.x
        v_rel = best_det.velocity_mps.x

        # Low-pass filter relative velocity
        self.v_rel_filt = (
            self.alpha * v_rel +
            (1.0 - self.alpha) * self.v_rel_filt
        )

        # v_lead = self.v_ego + self.v_rel_filt

        # Debug print statements 
        rospy.loginfo("Lead detected | d_lead = %.2f m | v_rel = %.2f m/s",
                      d_lead, self.v_rel_filt)
        
        

        self.pub_lead_dist.publish(d_lead)
        self.pub_lead_rel_vel.publish(self.v_rel_filt)

if __name__ == "__main__":
    try:
        RadarLeadExtractor()   # <-- THIS LINE WAS MISSING
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

