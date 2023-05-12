Mapping mode ends by writing map to /home/amy/amyos/mymap

Navigation mode starts by reading this map (navigation/teb_local_planner_tutorials/launch/3_teb_with_fused_odom_m.launch)
This mentions amcl, investigate further
Android tablet must have some way of putting map files on Happy, scp?, rosbridge tcp?

When nav started, map tf is started.
rosrun tf tf_echo map odom
Which frame does /odom belong to?
Have a look at /amcl_pose when nav running

mapon starts:
/home/amy/amyos/src/navigation/amy_navigation/launch/fused_odom_m.launch
wait 2 seconds
/home/amy/amyos/src/navigation/amy_navigation/launch/includes/sick_gmapping.launch.xml
gmapping doesn't allow for starting from an initial map

save map calls:
rosrun map_server map_saver -f /home/amy/amyos/mymap

navs starts:
/home/amy/amyos/src/navigation/amy_navigation/launch/3_navigation_m.launch
amy_comm::CmdLed led_cmd_data;
led_cmd_data.Cmd = 5;
led_cmd_data.Value = 2;
led_cmd_pub.publish(led_cmd_data);

 while not rospy.is_shutdown():
                        try:
                                now = rospy.Time.now()
                                listener.waitForTransform("/map", "/base_link", now, rospy.Duration(1.0))
                                (trans,rot) = listener.lookupTransform('/map', '/base_link', now)
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
                                # rospy.logerr("%s: map to baselink error!!!" % self.nodename)
                                pass
                        else:
                                location=PSender()
                                location.x=trans[0]
                                location.y=trans[1]
                                quaternion = (
                                        rot[0],
                                        rot[1],
                                        rot[2],
                                        rot[3])
                                euler = tf.transformations.euler_from_quaternion(quaternion)
                                location.z = euler[2]
                                self.pose_pub.publish(location)
                        rate.sleep()

rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS \"{\'max_vel_x\':" + std::to_string(d_tmp) + "}\"";



/led - changes to led state from amy control board
/CmdLed - control led

#SL - Publishes a movement to cmd_vel what repeats for a short while
#DANC - Dance mode
#FOLL - Follow mode
#VL - Published a movement to cmd_vel
#CANCEL - Cancel current navigation
#MAPO - Enables mapping (this turns off navigation if it is on)
#QMAP - Stop mapping
#SMAP - Save map - rosrun map_server map_saver -f /home/amy/amyos/mymap
#GBAT - Get battery info
#GEBS - Get emergency stop status
#NAVS - Start navigation (stops mapping if it is on)
#PT! - Go to a location
#STFO - Stop dancing or following
#TURN - Turn
#STOP - Stop navigating
#INPO! - Set the initial position
SHUTDOWN - SHUTDOWN
#DOCK - Dock the robot
#CDCK - Cancel docking
#GDFLAG - Get dock status
#GNFLAG - Get navigation status
#MACC - Checks whether a map exists
VERBO - Returns driver firmware version
VERAC - Returns data firmware version
VERSY - Returns robot version
CVERA - Returns the above versions in one go
UPDDR - Update the robot
#CPUO - Stress tests the CPU
#CPUS - Kills the stress test
#SSCC! - Sets safety control parameters
#SSPD! - Sets the maximum velocity for navigation
#GSPD - Gets the maxiumum velocity for navigation
#GOLF - Gets the overload flag status (no idea what this is, but comes from topic /overload_flag which is never published by our model)
#REOL - Reset overload flag
#RANP - Reset Android power
#BJZ:START -
#BJZ:STOP -
#BJZ:DOCK - Something to do with Mag (magnetic) navigation, not used on our model

Map save locations
/home/amy/amyos/mymap.yaml
/home/amy/amyos/mymap.pgm
/home/amy/amyos/location.txt
