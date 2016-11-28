import string
import math

def calc_ik(x,y,z):
	#lengths of links
	l_1 = 206.0
	l_2 = 190.0
	l_3 = 220.0
	l_4 = 45.0

	joint_vals = [0.0] * 4

	# position of the "wrist"
	z_prime = z+45.0

	# distance from the top of the end of the "waist" joint
	dd = math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow((z_prime-l_1),2))

	# phi is the angle from normal to the position of the wrist
	phi = math.asin((z_prime-l_1)/dd)

	#angle about joint 2 to the line from the top of the "waist" to the "wrist"
	C_1 = math.acos((math.pow(l_3,2)-math.pow(dd,2)-math.pow(l_2,2))/(-2.0*dd*l_2))

	#angle about joint 3 between links 2 and 3
	C_2 = math.acos((math.pow(dd,2)-math.pow(l_3,2)-math.pow(l_2,2))/(-2.0*l_3*l_2))

	joint_vals[0] = math.atan2(y,x)
	joint_vals[1] = math.radians(90.0)-phi-C_1
	joint_vals[2] = math.radians(180.0)-C_2
	joint_vals[3] = math.radians(180.0)-(math.radians(180.0)-C_1-C_2)-(math.radians(180.0)-math.radians(90.0)-phi)

	return joint_vals

if __name__=='__main__':
	input_string = raw_input('Please enter the desired end effector position (in cm) to move to deliminated by spaces-->')
	xyz_vals = string.split(input_string,' ')
	if len(xyz_vals) < 3:
		print 'TOO FEW NUMBERS FOR COORDINATES IN 3D!!'
	else:
		joint_vals = calc_ik(float(xyz_vals[0])*10.0, float(xyz_vals[1])*10.0, float(xyz_vals[2])*10.0)
		print "The following are the calculated values for the joint values"
		joint_vals_degrees = [0.0]*4
		joint_vals_degrees[0] = math.degrees(joint_vals[0])
		joint_vals_degrees[1] = math.degrees(joint_vals[1])
		joint_vals_degrees[2] = math.degrees(joint_vals[2])
		joint_vals_degrees[3] = math.degrees(joint_vals[3])
		print joint_vals_degrees