import rospy
import tf
import tf_conversions.posemath as tfconv
import PyKDL
from compute_ik import forward
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick

class HandEyeConnector(object):
	"""docstring for HandEyeConnector"""
	def __init__(self):

		self.sample_rate = rospy.get_param('~sample_rate')
		self.interactive = rospy.get_param('~interactive')

		# Compute the camera base to optical transform, 
		# extensible to the case where camera base and optical is not alaigned.
		self.xyz_optical_camera = rospy.get_param('~xyz_optical_camera', [0,0,0])
		self.rpy_optical_camera = rospy.get_param('~rpy_optical_camera', [0,0,0])
		self.F_optical_camera 	= PyKDL.Frame(
			PyKDL.Rotation.RPY(*self.rpy_optical_camera),
			PyKDL.Vector(*self.xyz_optical_camera))
		self.F_camera_optical = self.F_optical_camera.Inverse()

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
			joint_angles = rospy.wait_for_message('/joint_states', JointState)
			TMatrix = forward(joint_angles.position)
			F_base_ee = tfconv.fromMatrix(TMatrix)
			F_ee_base = F_base_ee.Inverse()
			eb = tfconv.toMsg(F_ee_base)
 			trans = (eb.position.x, eb.position.y, eb.position.z)
 			rot = (eb.orientation.x, eb.orientation.y, eb.orientation.z, eb.orientation.w)
 			print(trans, rot)

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

 		n_min=2
 		if len(self.world_to_hand_samples.transforms) < n_min:
 			rospy.logwarn("%d more samples needed ..." % (n_min - len(self.world_to_hand_samples.transforms)))
 		else:
 			self.compute_calibration(msg)

 		
 		# Interactive
 		if self.interactive:
 			i = raw_input('Hit [Enter] to accept this latest sample, or "d" to discard:')
 			if i == 'd':
 				del self.world_to_hand_samples.transforms[-1]
 				del self.camera_to_marker_samples.transforms[-1]
 				self.compute_calibration(msg)
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

 		return result