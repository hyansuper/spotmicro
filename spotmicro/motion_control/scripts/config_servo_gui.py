#!/usr/bin/env python
from Tkinter import *
import ttk, rospy
from motion_control.srv import EnableServo, ConfigServo
from motion_control.msg import ServoConfig
from std_msgs.msg import Float64MultiArray

hardware_ns = "servo_pwm_controller/"
limits_ns = "joint_limits/"
config_ns = hardware_ns+"servo_config/"
joints = rospy.get_param("joint_position_controller/joints")
joint_positions = []

def update_attr(obj, attrs):
	for k,v in attrs.items():
		setattr(obj, k, v)

class Application(Frame):
	def __init__(self, master=None):
		Frame.__init__(self, master)
		self.pack()
		self.createRefWidgets({'PI':3.14159265359, 'PI/2':1.57079632679, 'PI/4':0.78539816339, 'PI*3/4':2.35619449, 'PI/6':0.52359877559}).grid(row=0)
		self.createServoWidgets().grid(row=1)
		f = Frame(self)
		f.grid(row=2)

	def createRefWidgets(self, d):
		f = Frame(self)
		r = 0
		for k,v in d.items():
			Label(f, text=k).grid(row=r//2, column=r%2*2)
			Entry(f, state='readonly', textvariable=StringVar(value=v)).grid(row=r//2, column=r%2*2+1)
			r += 1
		ttk.Separator(f, orient=HORIZONTAL).grid(row=(r+1//2), columnspan=4, sticky='ew')
		return f

	def createServoWidgets(self):
		f = Frame(self)
		for i, j in enumerate(joints):			
			self.ServoWidget(j, master=f).grid(row=i)
		return f

	class ServoWidget(Frame):

		def enable_servo(self):
			try:
				res = rospy.ServiceProxy(hardware_ns+'enable_servo', EnableServo)(self.joint, bool(self.var_enabled.get()))
				print(res)
			except rospy.ServiceException as e:				
				print("Service call failed: %s"%e)

		def set_servo(self, ev):
			try:
				sc = ServoConfig()
				sc.joint = self.joint
				sc.pwm_start = int(self.var_pwm_start.get())
				sc.pwm_end = int(self.var_pwm_end.get())
				sc.channel = int(self.var_channel.get())
				sc.board = int(self.var_board.get())
				res = rospy.ServiceProxy(hardware_ns+'config_servo', ConfigServo)(sc)
				print(res)
				if res.success:
					msg = Float64MultiArray()
					msg.data = [float(pos.get()) for pos in joint_positions]
					pub.publish(msg)
			except rospy.ServiceException as e:
				print("Service call failed: %s"%e)


		def __init__(self, joint, master=None):
			Frame.__init__(self, master)
			self.joint = joint
			self.var_enabled = IntVar()
			Checkbutton(self, text=joint, variable=self.var_enabled, command=self.enable_servo).grid(row=0, column=0)
			ttk.Separator(self, orient=VERTICAL).grid(row=0, column=1, rowspan=2, sticky="sn")
			Label(self, text="min_pos").grid(row=0, column=2)
			Entry(self, state='readonly', textvariable=StringVar(value=rospy.get_param(limits_ns+joint+"/min_position"))).grid(row=0, column=3)
			Label(self, text="max_pos").grid(row=0, column=4)
			Entry(self, state='readonly', textvariable=StringVar(value=rospy.get_param(limits_ns+joint+"/max_position"))).grid(row=0, column=5)

			self.var_position = StringVar(value="0")
			joint_positions.append(self.var_position)
			Label(self, text="position").grid(row=0, column=6)
			e = Entry(self, textvariable=self.var_position)
			e.grid(row=0, column=7)
			e.bind("<Return>", self.set_servo)

			self.var_pwm_start = StringVar(value=rospy.get_param(config_ns+joint+"/pwm_start"))
			Label(self, text="pwm_start").grid(row=1, column=2)
			e = Entry(self, textvariable=self.var_pwm_start)
			e.grid(row=1, column=3)
			e.bind("<Return>", self.set_servo)

			self.var_pwm_end = StringVar(value=rospy.get_param(config_ns+joint+"/pwm_end"))
			Label(self, text="pwm_end").grid(row=1, column=4)
			e = Entry(self, textvariable=self.var_pwm_end)
			e.grid(row=1, column=5)
			e.bind("<Return>", self.set_servo)

			self.var_board = StringVar(value=rospy.get_param(config_ns+joint+"/board"))
			Label(self, text="board").grid(row=1, column=6)
			e = Entry(self, textvariable=self.var_board)
			e.grid(row=1, column=7)
			e.bind("<Return>", self.set_servo)

			self.var_channel = StringVar(value=rospy.get_param(config_ns+joint+"/channel"))
			Label(self, text="channel").grid(row=1, column=8)
			e = Entry(self, textvariable=self.var_channel)
			e.grid(row=1, column=9)
			e.bind("<Return>", self.set_servo)			
			ttk.Separator(self, orient=HORIZONTAL).grid(row=2, columnspan=10, sticky="ew")

rospy.init_node('config_servo_gui')
pub = rospy.Publisher('joint_position_controller/command', Float64MultiArray, queue_size=1)
root = Tk()
root.title('configure servo')
app = Application(root)
root.mainloop()
root.destroy()