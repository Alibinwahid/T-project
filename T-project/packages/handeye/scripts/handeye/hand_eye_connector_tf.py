import rospy
import tf
import tf_conversions.posemath as tfconv
import PyKDL
import yaml

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick

class HandEyeConnectorTF(object):
	"""docstring for HandEyeConnector"""
	def __init__(self):
		# The transform from this frame to the camera optical frame is what
        # we're trying to compute.
        # For the eye-in-hand case, this is the end-effector frame.
        # For the eye-on-base case, this is the world or base frame.
		self.camera_parent_frame_id = rospy.get_param('~camera_parent_frame')

		# Frame which is rigidly attached to the marker
        # The transform from the camera parent frame to the marker parent frame
        # is given by forward kinematics.
        # For the eye-in-hand case, this is the world or base frame.
        # For the eye-on-base case, this is the end-effector frame.
		self.marker_parent_frame_id = rospy.get_param('~marker_parent_frame')

		self.sample_rate = rospy.get_param('~sample_rate')
		self.interactive = rospy.get_param('~interactive')
		self.sample_num  = rospy.get_param('~sample_num')
		self.data_file_name = rospy.get_param('~data_file_name')

		# Compute the camera base to optical transform, 
		# extensible to the case where camera base and optical is not alaigned.
		self.xyz_optical_camera = rospy.get_param('~xyz_optical_camera', [0,0,0])
		self.rpy_optical_camera = rospy.get_param('~rpy_optical_camera', [0,0,0])
		self.F_optical_camera 	= PyKDL.Frame(
			PyKDL.Rotation.RPY(*self.rpy_optical_camera),
			PyKDL.Vector(*self.xyz_optical_camera))
		self.F_camera_optical = self.F_optical_camera.Inverse()

		self.listener = tf.TransformListener()

		# Rate limiter
		self.rate = rospy.Rate(self.sample_rate)

		# Input Date 
		self.world_to_hand_samples = TransformArray()
		self.camera_to_marker_samples = TransformArray()

		raw_input('Hit [ENTER] to start to calibrate...')

		# Calibration service
		rospy.wait_for_service('compute_effector_camera_quick')
		self.calibrate = rospy.ServiceProxy(
			'compute_effector_camera_quick', 
			compute_effector_camera_quick)

		# Marker subscriber
		self.aruco_subscriber = rospy.Subscriber(
			'aruco_tracker/transform',
			TransformStamped,
			self.aruco_cb,
			queue_size=1)


	def aruco_cb(self, msg):
		rospy.loginfo('Received marker sample.')

		# Get the camera optical frame
		optical_frame_id = msg.header.frame_id

		try:
			# Get the transform from the marker to camera frames with Forward Kinematics
			self.listener.waitForTransform(
				self.marker_parent_frame_id, self.camera_parent_frame_id,
				msg.header.stamp, rospy.Duration(0.1))

			(trans, rot) = self.listener.lookupTransform(
				self.marker_parent_frame_id, self.camera_parent_frame_id,
				msg.header.stamp)

		except tf.Exception as ex:
			rospy.logwarn(str(ex))
			return
 		
 		# Update data
 		self.world_to_hand_samples.header.frame_id = optical_frame_id
 		self.world_to_hand_samples.transforms.append(Transform(Vector3(*trans), Quaternion(*rot)))

 		self.camera_to_marker_samples.header.frame_id = optical_frame_id
 		self.camera_to_marker_samples.transforms.append(msg.transform)

 		if len(self.world_to_hand_samples.transforms) != len(self.camera_to_marker_samples.transforms):
 			rospy.logerr("Different numbers of world-to-hand and camera-to-marker samples.")
 			return

 		if len(self.world_to_hand_samples.transforms) < self.sample_num:
 			rospy.logwarn("%d samples collected, %d more samples needed ..." % (len(self.world_to_hand_samples.transforms), self.sample_num - len(self.world_to_hand_samples.transforms)))
 		else:
 			self.compute_calibration(msg)
 			self.aruco_subscriber.unregister()
 			print("Calibration finished. Wait for Termination.")
 		
 		# Interactive
 		if self.interactive:
 			i = raw_input('Hit [Enter] to accept this latest sample, or "d" to discard:')
 			if i == 'd':
 				del self.world_to_hand_samples.transforms[-1]
 				del self.camera_to_marker_samples.transforms[-1]
 			raw_input('Hit [Enter] to capture the next sample')
 		else:
 			self.rate.sleep()

 	def compute_calibration(self, msg):
 		rospy.loginfo("Computing from %d poses..." % len(self.world_to_hand_samples.transforms))
 		result = None

 		# Get the camera optical frame
 		optical_frame_id = msg.header.frame_id

 		try:
 			result = self.calibrate(self.camera_to_marker_samples, self.world_to_hand_samples)
 		except rospy.ServiceException as ex:
 			rospy.logerr("Calibration failed: " + str(ex))
 			return None

 		rospy.loginfo("Result:\n" + str(result))

 		ec = result.effector_camera
 		xyz = (ec.translation.x, ec.translation.y, ec.translation.z)
 		xyzw = (ec.rotation.x, ec.rotation.y, ec.rotation.z, ec.rotation.w)
 		rpy = tuple(PyKDL.Rotation.Quaternion(*xyzw).GetRPY())

 		F_optical_world = PyKDL.Frame(PyKDL.Rotation.Quaternion(*xyzw), PyKDL.Vector(*xyz))
 		F_camera_world = F_optical_world * self.F_camera_optical

 		cw = tfconv.toMsg(F_camera_world)
 		xyz = (cw.position.x, cw.position.y, cw.position.z)
 		xyzw = (cw.orientation.x, cw.orientation.y, cw.orientation.z, cw.orientation.w)
 		rpy = tuple(PyKDL.Rotation.Quaternion(*xyzw).GetRPY())

 		rospy.loginfo("Base xyz: (%f, %f, %f) rpy: (%f, %f, %f) xyzw: (%f, %f, %f, %f)" % (xyz+rpy+xyzw))

 		data = dict(
 			position = dict(
 				x = cw.position.x,
 				y = cw.position.y,
 				z = cw.position.z),
 			orientation = dict(
 				x = cw.orientation.x,
 				y = cw.orientation.y,
 				z = cw.orientation.z,
 				w = cw.orientation.w)
 			)
 		with open(self.data_file_name, 'w') as outfile:
 			yaml.dump(data, outfile, default_flow_style=False)

 		return result
