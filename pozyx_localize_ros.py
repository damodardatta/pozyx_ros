#!/usr/bin/env python3
import csv
import rospy
from std_msgs.msg import Float64
from time import sleep
from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, PozyxRegisters)
from pythonosc.udp_client import SimpleUDPClient
from pypozyx.tools.version_check import perform_latest_version_check
from math import pi,radians
import pypozyx
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Ready to range example: pozyx online docs | Integrated with ROS to record position with timestamp
# Recalibrate positions of all anchors using the Pozyx Creator Controller and 
# update the values in the script before recording any data
# Package requirements: pypozyx and python-osc


# Configuring setup parameters
ANCHOR_01_ID = 0x1106   # ORIGIN
ANCHOR_02_ID = 0x114D
ANCHOR_03_ID = 0x1176
ANCHOR_04_ID = 0x122D

# Requires calibration before every new test (when configuration may have changed)
ANCHOR_01_COORDINATES = Coordinates(-2, 1, 145)
ANCHOR_02_COORDINATES = Coordinates(998, 3449, 2175)
ANCHOR_03_COORDINATES = Coordinates(5499, 5, 1680)
ANCHOR_04_COORDINATES = Coordinates(5435, 2108, 210)
ANCHOR_COORDINATES = [ANCHOR_01_COORDINATES, ANCHOR_02_COORDINATES, ANCHOR_03_COORDINATES, ANCHOR_04_COORDINATES]

class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""

    def __init__(self, pozyx, osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id

        self.state_array = []
        self.current_state = {'ros_time': 0.0, 'pozyx_x': 0.0, 'pozyx_y': 0.0, 'pozyx_z': 0.0}
        self.pozyx_pose_publisher = rospy.Publisher('/pozyx_pose', Pose, queue_size=40)
        self.x_msg = Pose()

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")
        print("- System will manually configure tag")
        print("")
        print("- System will auto start positioning")
        print("")
        if self.remote_id is None:
            self.pozyx.printDeviceInfo(self.remote_id)
        else:
            for device_id in [None, self.remote_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")

        self.setAnchorsManual(save_to_flash=False)
        self.printPublishConfigurationResult()

        rospy.init_node('pozyx_positioning')
        rospy.loginfo('Initialized ROS node.')

    def loop(self):
        """Performs positioning and displays/exports the results."""

        position = Coordinates()
        quat = Quaternion()
        # orientation = pypozyx.EulerAngles()
        quat_data = pypozyx.Quaternion()
        status = self.pozyx.getQuaternion(quat_data, remote_id=self.remote_id)
        # status = self.pozyx.getEulerAngles_deg(orientation, remote_id=remote_id)
        # rospy.loginfo(orientation)
        # quat = quaternion_from_euler(-radians(orientation.pitch), radians(orientation.roll), -radians(orientation.heading))
        rospy.loginfo(euler_from_quaternion([quat_data.x,quat_data.y,quat_data.z,quat_data.w]))
        status &= self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            current_state = {'ros_time': 0.0, 'pozyx_x': 0.0, 'pozyx_y': 0.0, 'pozyx_z': 0.0}
            current_state['ros_time'] = rospy.Time.now().to_time()
            # print('Recorded value pox_x: {}'.format(current_state['pozyx_x']))
            current_state['pozyx_x'] = position.x
            current_state['pozyx_y'] = position.y
            current_state['pozyx_z'] = position.z
            current_state['pozyx_q1'] = quat_data.x     
            current_state['pozyx_q2'] = quat_data.y     
            current_state['pozyx_q3'] = quat_data.z     
            current_state['pozyx_q4'] = quat_data.w
            global prev_y,prev_yf,prev_x,prev_z,prev_zf,prev_xf,i

            self.x_msg = position.x
            self.y_msg = position.y
            self.z_msg = position.z             
            
            #writing data to current state after low pass
            current_state['pozyx_x'] = self.x_msg
            current_state['pozyx_y'] = self.y_msg
            current_state['pozyx_z'] = self.z_msg
            
            
            # Adds to array if unique state entry (duplicate prevention)
            if not current_state in self.state_array:
                self.state_array.append(current_state)
                self.pozyx_pose_publisher.publish(Point(self.x_msg, self.y_msg, self.z_msg),Quaternion(quat_data.x, quat_data.y, quat_data.z, quat_data.w))
            # rospy.loginfo('Pozyx positioning x: {} y: {} z: {}'.format(self.x_msg, self.y_msg, self.z_msg))
        else:
            self.printPublishErrorCode("positioning")
        

    def printPublishPosition(self, position):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        network_id = self.remote_id
        if network_id is None:
            network_id = 0
        print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
            "0x%0.4x" % network_id, pos=position))
        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, int(position.x), int(position.y), int(position.z)])

    def printPublishErrorCode(self, operation):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, -1])
            # should only happen when not being able to communicate with a remote Pozyx.

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(remote_id=self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors), remote_id=self.remote_id)

        if save_to_flash:
            self.pozyx.saveAnchorIds(remote_id=self.remote_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
        return status

    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [device_list[i], int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)

    def printPublishAnchorConfiguration(self):
        """Prints and potentially publishes the anchor configuration"""
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, int(anchor.coordinates.x), int(anchor.coordinates.y), int(anchor.coordinates.z)])
                sleep(0.025)

if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = False   # Set to false as installation is recent and latest
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6839                 # remote device network ID
    remote = True                      # whether to use a remote device
    if not remote:
        remote_id = None

    # enable to send position data through OSC
    use_processing = True

    # configure if you want to route OSC to outside your localhost. Networking knowledge is required.
    ip = "127.0.0.1"
    network_port = 8888

    osc_udp_client = None
    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)

    # necessary data for calibration, change the IDs and coordinates yourself according to your measurement
    anchors = [DeviceCoordinates(ANCHOR_01_ID, 1, ANCHOR_01_COORDINATES),
               DeviceCoordinates(ANCHOR_02_ID, 1, ANCHOR_02_COORDINATES),
               DeviceCoordinates(ANCHOR_03_ID, 1, ANCHOR_03_COORDINATES),
               DeviceCoordinates(ANCHOR_04_ID, 1, ANCHOR_04_COORDINATES)]

    # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_TRACKING #UWB_ONLY
    # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
    dimension = PozyxConstants.DIMENSION_3D
    # height of device, required in 2.5D positioning
    height = 0

    pozyx = PozyxSerial(serial_port)
    r = ReadyToLocalize(pozyx, osc_udp_client, anchors, algorithm, dimension, height, remote_id)
    r.setup()
    logging = True
    while not rospy.is_shutdown():
        r.loop()
    
    rospy.loginfo('Node shutdown called.')
    logIndex = str(121004)
    print(r.current_state)
    csv_filename = 'pozyxLog' + logIndex + '.csv'
    fields = ['ros_time', 'pozyx_x', 'pozyx_y', 'pozyx_z','pozyx_q1' ,'pozyx_q2' ,'pozyx_q3' ,'pozyx_q4']
    rospy.loginfo('Data samples collected : {}. Writing to csv file {}'.format(len(r.state_array), csv_filename))

    with open('/home/damodar/rrc-indoor-navigation-airbus/mocap_catkin_ws/csvlogs/damodar/pozyx/PozyxLog-'+logIndex+'.csv', 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fields)
        writer.writeheader()
        writer.writerows(r.state_array)
    rospy.loginfo("Finished writing to file. Exiting...")