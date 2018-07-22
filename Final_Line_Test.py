# Import required libraries
import cv2
import numpy as np
from picamera import PiCamera
import time
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from time import sleep




################## Line Follower Functions #################

# Motor Pins

# Refers to the pins connected to motor in Raspberry pi
left_motorA = 38	# Referred as forward_left
left_motorB = 36	# Referred as reverse_left
left_motorE = 40
right_motorA = 32	# Referred as forward_right
right_motorB = 35	# Referred as reverse_right
right_motorE = 37
Trigger = 18
Echo = 22

GPIO.setwarnings(False)		# To disable any warning provided by the board
GPIO.setmode(GPIO.BOARD)	# Set mode of GPIO board to access gpio pins by their pin number on board
GPIO.setup(left_motorA,GPIO.OUT) 	# Setup GPIO pins to output mode
GPIO.setup(left_motorB,GPIO.OUT)
GPIO.setup(left_motorE,GPIO.OUT)
GPIO.setup(right_motorA,GPIO.OUT)
GPIO.setup(right_motorB,GPIO.OUT)
GPIO.setup(right_motorE,GPIO.OUT)
GPIO.setup(Trigger,GPIO.OUT)
GPIO.setup(Echo, GPIO.IN)

left_DutyA = GPIO.PWM(left_motorA, 100)		# PWM working on ACW and CW pins of the motors rather than the enable pin
right_DutyA = GPIO.PWM(right_motorA, 100)
left_DutyB = GPIO.PWM(left_motorB,100)
right_DutyB = GPIO.PWM(right_motorB,100)

"""
* Function Name : speed_control
* Input : Duty - Used to change the speed of motor turning using PWM
* Output : Void
* Logic : PWM used to change the speed of motor when needed by changing the Duty Cycle
* Example Call : speed_control(40)- turns the forward motors at 40% of the maximum possible speed. 
"""
def speed_control(Duty):
	left_DutyA.ChangeDutyCycle(Duty)		# Since speed control is need only when robot is moving forward, only duty cycle for forward motors needs to be changed
	right_DutyA.ChangeDutyCycle(Duty)
	#left_DutyB.ChangeDutyCycle(Duty)
	#right_DutyB.ChangeDutyCycle(Duty)


"""
* Function Name : make_sharp_turn
* Input : roi_img -- ROI masked binary image		
* Output : Void
* Logic : To check whether the robot has to take a sharp left or sharp right turn , masked ROI image is check to find the contours of maximum area ( the black line ) and its bounding rectangle is found .
	  If the bounding rectangle has very small x and y co-ordinates , this means it is a sharp left turn , else it's sharp right turn.

* Example Call : make_sharp_turn(roi_mask)
"""
def make_sharp_turn(roi_img):
	x,y = roi_img.shape
	print x,y
	_,contours,heirarchy = cv2.findContours(roi_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)		# Find contours of the masked image

	if len(contours)>0:	# Boundary Check
		# To find contour with maximum area i.e. Black track
		ci=0
		max_area=-1
		for i in range(len(contours)):
			area=cv2.contourArea(contours[i])
			if area>max_area and area>200:	# Neglect noisy parts of image
				ci=i		

		bx,by,w,h=cv2.boundingRect(contours[ci])	# Calculate bounding rectangle of the black line
		print bx,by
		if (bx<=10 and by<=10) :		# If co-ordinates of the bounding rectangle are small , it's a left sharp turn
			print "Left sharp turn"	
			sharp_left()	# Call function to take the sharp turn using differential motor control
	  		# On sharp turn , speed of motor is very high due to differential motor control , so manually stop the motor after small interval of time, else it might lose track the black track
			sleep(0.75)	# Call function to take turn for 0.5 seconds
			stop()		# Call function to stop the motor

		elif bx>=100:		# Else it is a sharp right turn
			print "Right sharp turn"
			sharp_right()	# Call function to take sharp right turn
			sleep(0.75)	# Stop motor after small time . Motor will turn enough to think of the turn as normal curved turn instead of sharp turn.
			stop()		# Call function to stop the motor
		else :
			print "False Turn"
			pass


"""
* Function Name : sharp_left
* Input : None
* Output : Void
* Logic : Uses differential motor control action for sharp turns i.e one motor (right) moves forward and other (left) moves backward
* Example Call : sharp_left() - Left motor moves backward and right motor moves forward
"""
def sharp_left():
	left_DutyA.stop()	# Stop forward_left
	left_DutyB.start(35)	# Start reverse_left with 35% of total speed  
	right_DutyB.stop()	# Stop reverse_right
	right_DutyA.start(35)	# Start forward_right with 35% of total speed 

"""
* Function Name : sharp_right
* Input : None
* Output : Void
* Logic : Uses differential motor control action for sharp turns i.e one motor (right) moves backward and other (left) moves forward 
* Example Call : sharp_right() - Right motor moves forward and left motor moves backward
"""
def sharp_right():
	left_DutyB.stop()	# Stop reverse_left
	left_DutyA.start(35)	# Start forward_left with 35% of total speed
	right_DutyA.stop()	# Stop forward_left
	right_DutyB.start(35)	# Start reverse_left with 35% of total speed

"""
* Function Name : forward
* Input : None
* Output : Void
* Logic : Used to set gpio pins referring to forward motor so that robot moves in forward direction
* Example Call : forward() - Left and right motors move in forward direction
"""
def forward():
	left_DutyB.start(0)		# Start reverse_left motor with 0 speed 
	right_DutyB.start(0)		# Start forward_right with 0 speed 	( Used so that any previous state of the forward_right motor does no affect the current state)
	left_DutyA.start(30)		# Start forward_left motor with 30 speed
	right_DutyA.start(30)		# Bot goes forward
	#GPIO.output(left_motorA,GPIO.HIGH)	# Set respective pins to high 
        #GPIO.output(right_motorA,GPIO.HIGH)
	#GPIO.output(left_motorB,GPIO.LOW)
	#GPIO.output(right_motorB,GPIO.LOW)

"""
* Function Name : backward
* Input : None
* Output : Void
* Logic : Used to set gpio pins referring to forward motor so that robot moves in backward direction
* Example Call : backward() - Left and right motors move in forward direction
"""
def backward():
	# Used in case bot goes off course a little bit and does not detect any black line contours , it can go backward to the track again
	left_DutyA.stop()	# Stop forward_left 
	right_DutyA.stop()	# Stop reverse_left
	left_DutyB.start(20)	# Start reverse_left motor with 20 speed 
	right_DutyB.start(20)	# Bot moves backward
	#GPIO.output(left_motorB,GPIO.HIGH)	# Set respective pins to high 
        #GPIO.output(right_motorB,GPIO.HIGH)
	#GPIO.output(left_motorA,GPIO.LOW)
	#GPIO.output(right_motorA,GPIO.LOW)
"""
* Function Name : right
* Input : None
* Output : Void
* Logic : Used to set gpio pins referring to motor so that robot moves in right direction
* Example Call : right() - Left motor moves forward and right motor moves reverse with small speed to allow smaller turning radius
"""
def right():
	left_DutyB.stop()	# Stop reverse_left
	left_DutyA.start(30)	# To move right, forward_left is set to high
	right_DutyA.stop()	# Stop forward_left
	right_DutyB.start(0)	# Reverse_right set to quite lower speed, allows small turning radius for the robot
	#sleep(0.01)
	#GPIO.output(left_motorA,GPIO.HIGH)	# Set respective pins to high 
        #GPIO.output(right_motorA,GPIO.LOW)
	#GPIO.output(left_motorB,GPIO.LOW)
	#GPIO.output(right_motorB,GPIO.LOW)

"""
* Function Name : left
* Input : None
* Output : Void
* Logic : Used to set gpio pins referring to forward motor so that robot moves in left direction
* Example Call : left() - Left motor moves reverse with small speed and right motor moves forward to allow smaller turning radius
"""
def left():
	left_DutyA.stop()	# Stop forward_left
	left_DutyB.start(0)	# Reverse_left is set to lower speed, to allow small turning radius
	right_DutyB.stop()	# Stop reverse_left
	right_DutyA.start(30)	# To move left, forward_right is set to high
	#sleep(0.01)
	#GPIO.output(left_motorA,GPIO.LOW)	# Set respective pins to high 
        #GPIO.output(right_motorA,GPIO.HIGH)
	#GPIO.output(left_motorB,GPIO.LOW)
	#GPIO.output(right_motorB,GPIO.LOW)

"""
* Function Name : stop
* Input : None
* Output : Void
* Logic : Used to stop the robot by setting all motor pins to active low
* Example Call : stop() - Stop both left and right motors
"""
def stop():
	left_DutyA.stop()	# Stop the forward_left motor
	left_DutyB.stop()	# Stop the reverse_left motor
	right_DutyA.stop()	# Stop the forward_right motor
	right_DutyB.stop()	# Stop the reverse_right motor	(Completely stops both the motors)
	GPIO.output(left_motorA,GPIO.LOW)	# Set the respective gpio motor pins to low
	GPIO.output(left_motorB,GPIO.LOW)
	GPIO.output(right_motorA,GPIO.LOW)
	GPIO.output(right_motorB,GPIO.LOW)
	GPIO.output(left_motorE,GPIO.LOW)
	GPIO.output(right_motorE,GPIO.LOW)


"""
* Function Name : modulus
* Input : Two numbers num1 and num2
* Output : Returns the modulus of two numbers
* Logic : Returns the positive of the difference of the two numbers . Function used during direction control
* Example Call : modulus(10,20) returns 10
"""
def modulus(num1,num2):
	if num1>=num2:	# Number 1 is greater than number 2
		return (num1-num2)		#Return the modulus (postive of difference) of two numbers.
	else :	# Number 2 is greater than number 1
		return (num2-num1)


"""
* Function Name : direction_control

* Input : 1. current_roi_front - The centroid of the black line detected in the front part of roi
	  2. corner_roi_front - The leftmost point of the black line detected in the front part of roi
	  3. current_roi_bottom - The centroid of the black line detected in the bottom part of roi ( 2 ROIs are used to allow better control of the speed of the robot)
	  4. corner_roi_bottom - The leftmost point of the black line detected in the front part of roi

	Note : All the above values are only the x-coordinates of the points as y-coordinates are not needed for direction control

* Output : Void

* Logic : The function is used to check the direction in which the bot has to move currently . The distance between leftmost point of black line to the centroid of the line (of bottom ROI) is used to 	  follow the required direction, and the displacement between the leftmost points of the top and bottom ROI is used to control speed of forward motors... So, Front ROI is used for speed control 
	  and bottom ROI is used for direction control.

* Example Call : direction_control(200,120,205,125)

"""
def direction_control(current_roi_bottom,corner_roi_bottom):

	try:	# Executes if no expection occurs
		width = modulus(current_roi_bottom,corner_roi_bottom)	# Compute distance (modulus) between leftmost point and centroid of black line (Basically half of width of the black line)
		print width

		if (width>=38 and width<=42):	# Experimental Values for forward Direction -- Depend on the size of PiCam window
			print "Forward"
			forward()	# Call function to move motors in forward direction
			speed_control(20)	# Call function to control speed of motor

			#sleep(0.5)
			#stop()

		# Left or right direction depend on position of leftmost point from the center of the PiCam window ( Currently used window -- (480,320)
		elif corner_roi_bottom>=220:		# If leftmost point is to the right of the center of the picam window, robot moves to right
			print "Right"
			right()		# Call function to move the robot to right
			#sleep(0.5)
			#stop()

		elif corner_roi_bottom<220:
			print "Left"		# If leftmost point is to the left of the center of the picam window, robot moves to left
			left()		# Call function to move the robot to left
			#sleep(0.5)
			#stop()


	finally:	# Pass if exception occurs
		pass

"""
* Function Name : create_roi
* Input : 1. img - Input image captured by the picam
	  2. x1_roi , x2_roi - x-coordinates needed for ROI on the image 
	  3. y1_roi , y2_roi - y-coordinates needed for ROI on the image
	  4. thresh_type - Type of thresholding needed on the image ( BINARY_INV for black line detection and BINARY for inverted plane or white line detection ).

* Output : Returns a tuple so that the function returns multiple values. These include:
	   1. roi_img - Masked binary image needed to detect inverted planes.
	   2. bx - Leftmost point of the black line
	   3. cx - Centroid of the black line
	   4. max_area - Maximum area of black line needed to detect sharp turns and zone indicator

* Logic - The function creates ROI at the specified  co-ordinates values on the image by using contours approximation techniques. Blurring methods along with erosion techniques are used to reduce noise.
	  From the said contours , maximum area contour is obtained so that black line can be detected with precision.

* Example Call - create_roi(img,x-50,x-25,y/20,y-y/20,1) - where x and y are the dimensions of the img captured by picam and 0 represents thresholding using BINARY_INV technique.
"""
def create_roi(img,x1_roi,x2_roi,y1_roi,y2_roi,thresh_type):
	roi = img[x1_roi:x2_roi,y1_roi:y2_roi]		# Crop the image to get only ROI
	roi_gray=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)	# Convert to grayscale for accurate detection
	roi_gray=cv2.GaussianBlur(roi_gray,(5,5),0)	# Blur
	ret,roi_mask=cv2.threshold(roi_gray,70,255,thresh_type)	# Thresholding using required thresholding type BINARY or BINARY_INV as per black line or white line (inverted plane) traversal
	roi_mask=cv2.GaussianBlur(roi_mask,(5,5),0)	# Blur the threshed image
	roi_mask=cv2.medianBlur(roi_mask,5)	# Blur
	roi_mask=cv2.erode(roi_mask,np.array([15,15],np.uint8),iterations=1)	# Erosion to remove noise
	#cv2.imshow("ROI",roi_mask)
	#cv2.rectangle(img,(y1_roi,x1_roi),(y2_roi,x2_roi),(255,0,0),2)
	roi_img=roi_mask.copy()				# Make a copy of roi (so that it can be returned)as contour detection makes changes to the original image
	_,contours,heirarchy = cv2.findContours(roi_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)		# Contour detection

	if len(contours)>0:		#Boundary check
		# To find contour with maximum area
		ci=0
		max_area=-1
		for i in range(len(contours)):
			area=cv2.contourArea(contours[i])
			if area>max_area and area>1000:		# Boundary check
				ci=i

		max_area=cv2.contourArea(contours[ci])
		bx,by,w,h=cv2.boundingRect(contours[ci])		# Bounding rectangle to obtain the leftmost part of the black line
		#cv2.rectangle(roi,(bx,by),(bx+w,by+h),(0,255,0),2)		#Changes made to ROI are automatically reflected back to the original image (and vice versa). No need to set some offset

		# To find centroid of the black line using Moments
		M=cv2.moments(contours[ci])
		if M['m00'] != 0:		# Boundary check
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
		else :			# To avoid the "Division by zero" error if no contour is detected
			cx=0
			cy=0
		#cv2.circle(roi,(cx,cy),3,(0,0,255),-1)
		#cv2.imshow("Image",img)
		#cv2.waitKey(1000)
		return (roi_img,bx,cx,max_area)		# Returns a tuple 
	else :
		return (roi_img,0,0,0)		# If no contour is present, return threshed image, rest all values are zero.


"""
* Function Name : check_if_black
* Input : mask - Threshed image obtained from create_roi function
* Output : 1 (Image is black) or 0 (Image is False)
* Logic : The function checks if the whole threshed ROI image is black or not . Helps in the detection of the Zone Indicators
* Example Call : check_if_black(roi_mask) - where roi_mask is the threshed binary image
"""
def check_if_black(mask):
	# Take a bitwise NOT of the binary image . As for black line detection, we thresh black color, hence in binary image, black color turns to white and rest colors are black.
	mask = cv2.bitwise_not(mask)		
	if cv2.countNonZero(mask) == 0:		# Check if the now black image is Completely black or not
		print "Black Image"
		return 1	# If image is completely black
	else :
		return 0	# If image is not completely black




################## Interfacing	 #################
	

# Blue Mask
lower_blue = np.array([80,10,20])
upper_blue = np.array([130,255,255])

# Green Mask
lower_green = np.array([40,20,20])
upper_green = np.array([90,255,255])

# Red Mask
lower_red = np.array([0,130,90])
upper_red = np.array([20,255,255])

# Initialises the PiCamera
camera = PiCamera()
camera.resolution = (480,320)		# Set the resolution of PiCam
camera.framerate = 5		# Set the frame rate of PiCam
rawCapture = PiRGBArray(camera, size=(480,320))		# Capture raw Image
time.sleep(8)		# Time for PiCam to warm up
#gpio_set()		# Call function to setup GPIO pins
#skip_frames=0
check_zone=0		# Initially no zone is present
backward_turn_times=0
trigger=0

for frame in camera.capture_continuous(rawCapture, format = "bgr" , use_video_port=True):
	img=frame.array		# Capture the image

	x,y,c=img.shape		# Get the dimensions of the image captured

	#cv2.imshow("Plantation",plantation)
	if trigger==0:	
		GPIO.output(Trigger, GPIO.HIGH)
		time.sleep(0.00001)
		GPIO.output(Trigger, GPIO.LOW)
		trigger=1
		start_time=0
		stop_time=0

	if GPIO.input(Echo) == 0 and trigger==1 and stop_time==0:
		start_time = time.time()

	if GPIO.input(Echo) == 1 and trigger==1:
		stop_time = time.time()

	if start_time!=0 and stop_time!=0:
		time = start_time - stop_time
		print 17150*time
		trigger=0

	img=cv2.GaussianBlur(img,(5,5),0)	# Blur the image
	(roi_img,left_roi_1x,roi_1x,contour_area_1x)=create_roi(img,x-50,x-25,y/20,y-y/20,thresh_type=1)		# Call function to obtain the ROI at the bottom of the image
	print left_roi_1x,roi_1x,contour_area_1x
	#break
	if left_roi_1x == 0 and roi_1x == 0:
		print "Backwards"
		backward()
		sleep(0.5)
		stop()
		rawCapture.truncate(0)
		continue

	"""if contour_area_1x > 5000 :
		print "Sharp Turn"
		break
		forward()
		sleep(0.5)
		stop()
		make_sharp_turn(roi_img)
		rawCapture.truncate(0)
		continue

	else:"""
	direction_control(roi_1x,left_roi_1x)

	rawCapture.truncate(0)		# Flush out the current frame
		#if cv2.waitKey(1) == ord('q'):
			#break
cv2.destroyAllWindows()		#End of Program
