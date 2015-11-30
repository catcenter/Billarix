import socket
import Adafruit_BBIO.PWM as PWM

import time

PWM.start("P9_22", 0)
#time.sleep(1)
PWM.start("P9_21", 0)
#time.sleep(1)
PWM.start("P9_16", 0)
#time.sleep(1)
PWM.start("P9_14", 0)
#time.sleep(10)


   
UDP_IP = "192.168.43.114"
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))


#nominal value for pwm
nov = 30


yaw_old = 0
pitch_old = 0
roll_old = 0  

yaw_dot_old=0
pitch_dot_old = 0
roll_dot_old = 0

#integrations init
yawi = 0
pitchi  = 0
rolli = 0


#rate integrations
yawt = 0
pitcht  = 0
rollt = 0 




yaw_control_bool = 1 # zero for off yaw

kp_yaw = 0#.5
ki_yaw = 0
kd_yaw = 0#.001

kp_pitch = 0#1.1
ki_pitch = 0
Kd_pitch = 0#0.6

Kp_roll = 0#1.1
ki_roll = 0
kd_roll = 0#0.6

rate_feedback = 1 #or zero


krp_yaw = 0.0
kri_yaw = 0
krd_yaw = 0.001

krp_pitch = 1#0.8
kri_pitch = 0
Krd_pitch = 0#0.001

Krp_roll = 0#0.8
kri_roll = 0
krd_roll = 0#0.001


 
while True:
	data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes

	s = data
	x1 = s.find (',')		#first one for no. of readings
	x2 = s.find (',', x1 + 1)	#second one for time stamp
	x3 = s.find (',', x2 + 1)	#third one for yaw
	x4 = s.find (',', x3 + 1)	#fourth one for pitch
	x5 = s.find (',', x4 + 1)	#fifth one for roll
	

#angles
	yaw = float (s[x3+2:x4])
	pitch = float (s[x4+2:x5])
	roll = float ( s[x5+2:] )
	time = float (s[x2+2 : x3])
	#print (yaw, pitch,roll)
	#print (time)
	#print(s) #Time sample = 0.02 as in game mode (49 samples)

#rates of angles
	yaw_dot = (yaw-yaw_old)/0.02
	pitch_dot = (pitch - pitch_old)/0.02
	roll_dot = (roll - roll_old)/0.02

#Double rates of angles
	yaw_ddot = (yaw_dot - yaw_dot_old)/0.02	
	pitch_ddot = (pitch_dot - pitch_dot_old)/0.02
	roll_ddot = (roll_dot - roll_dot_old)/0.02
#Integrations
	yawi = yawi + yaw
	pitchi  = pitchi + pitch
	rolli = rolli + roll

	yawt = yawt + yaw_dot
	pitcht  = pitcht + pitch_dot
	rollt = rollt + roll_dot




# assign to olds
	yaw_old = yaw
	pitch_old = pitch
	roll_old = roll

	yaw_dot_old = yaw_dot
	pitch_dot_old = pitch_dot
	roll_dot_old = roll_dot


#Control signals U
	
#	uyaw = ((kp_yaw*yaw)+(ki_yaw*yawi)+(kd_yaw*yaw_dot))+((rate_feedback)*((krp_yaw*yaw_dot)+(kri_yaw*yawt)+(krd_yaw*yaw_ddot)))

	uyaw = 0

#	upitch = ((kp_pitch*pitch)+(ki_pitch*pitchi)+(Kd_pitch * pitch_dot))+((rate_feedback)*((krp_pitch*pitch_dot)+(kri_pitch*pitcht)+(Krd_pitch * pitch_ddot)))

#	uroll = ((Kp_roll*roll)+(ki_roll*rolli)+(kd_roll*roll_dot))+((rate_feedback)*((Krp_roll*roll_dot)+(kri_roll*rollt)+(krd_roll*roll_ddot)))
#Rate Control feedback
	upitch = ((kp_pitch*pitch)+(ki_pitch*pitchi)+(Kd_pitch * pitch_dot))+((rate_feedback)*((krp_pitch*pitch_dot)+(Krd_pitch * pitch_ddot)))


# PWM sigs
	pwm1 = nov - uroll    + uyaw
	pwm2 = nov + upitch   - uyaw
	pwm3 = nov + uroll    + uyaw
	pwm4 = nov - upitch   - uyaw

#	print(pwm1, pwm3, pwm2, pwm4)
	if pwm1 < 0:
		pwm1 = 0
	if pwm1 > 100:
		pwm1 = 100
	if pwm2 < 0:
		pwm2 = 0
	if pwm2 > 100:
		pwm2 = 100
	if pwm3 < 0:
		pwm3 = 0
	if pwm3 > 100:
		pwm3 = 100
	if pwm4 < 0:
		pwm4 = 0
	if pwm4>100:
		pwm4 = 100
	print("1: %5.2f 3: %5.2f 2: %5.2f 4: %5.2f roll: %6.2f pitch: %6.2f"%(pwm1, pwm3, pwm2, pwm4,uroll, upitch))
# PWM output pins
#Motor 1 (blank) (yellow)
	PWM.set_duty_cycle("P9_22",pwm1)
#motor 2 (Red) (White)
	PWM.set_duty_cycle("P9_21",pwm2)
#Motor 3 (Green) (purpul)
	PWM.set_duty_cycle("P9_14",pwm3)
#Motor 4 (Blue) (Green)
	PWM.set_duty_cycle("P9_16",pwm4)
