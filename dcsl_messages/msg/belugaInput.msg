#Actuator inputs for DCSL Beluga Robot

Header header

#Motor inputs are -255 to 255, positive values drive up and forwards respectively
int16 vertical_motor
int16 thrust_motor
#Servo inputs are -pi/2 TO pi/2, postive values rotate motor X when viewed from above
float32 servo
