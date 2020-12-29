#!/usr/bin/env python

""" GUI for Spot` controller"""

import rospy
from rs_msgs.msg import GaitInput
from Tkinter import Tk, Label, Button, Entry, END


class RsGui:
    def __init__(self, win, pub):
        self.lbl1 = Label(win, text='x')
        self.lbl2 = Label(win, text='y')
        self.lbl3 = Label(win, text='z')
        self.lbl4 = Label(win, text='roll')
        self.lbl5 = Label(win, text='pitch')
        self.lbl6 = Label(win, text='yaw')
        self.lbl7 = Label(win, text='StepLength')
        self.lbl8 = Label(win, text='LateralFraction')
        self.lbl9 = Label(win, text='YawRate')
        self.lbl10 = Label(win, text='StepVelocity')
        self.lbl11 = Label(win, text='ClearanceHeight')
        self.lbl12 = Label(win, text='PenetrationDepth')
        self.lbl13 = Label(win, text='SwingPeriod')
        self.lbl14 = Label(win, text='YawControl')
        self.lbl15 = Label(win, text='YawControlOn')
        self.t1 = Entry()
        self.t2 = Entry()
        self.t3 = Entry()
        self.t4 = Entry()
        self.t5 = Entry()
        self.t6 = Entry()
        self.t7 = Entry()
        self.t8 = Entry()
        self.t9 = Entry()
        self.t10 = Entry()
        self.t11 = Entry()
        self.t12 = Entry()
        self.t13 = Entry()
        self.t14 = Entry()
        self.t15 = Entry()
        self.t1.insert(END, 0)
        self.t2.insert(END, 0)
        self.t3.insert(END, 0.1)
        self.t4.insert(END, 0)
        self.t5.insert(END, 0)
        self.t6.insert(END, 0)
        self.t7.insert(END, 0.1)
        self.t8.insert(END, 0)
        self.t9.insert(END, 0)
        self.t10.insert(END, 0.8)
        self.t11.insert(END, 0.15)
        self.t12.insert(END, 0.00003)
        self.t13.insert(END, 0.3)
        self.t14.insert(END, 0.0)
        self.t15.insert(END, 1.0)
        self.lbl1.place(x=100, y=50)
        self.t1.place(x=220, y=50)
        self.lbl2.place(x=100, y=100)
        self.t2.place(x=220, y=100)
        self.lbl3.place(x=100, y=150)
        self.t3.place(x=220, y=150)
        self.lbl4.place(x=100, y=200)
        self.t4.place(x=220, y=200)
        self.lbl5.place(x=100, y=250)
        self.t5.place(x=220, y=250)
        self.lbl6.place(x=100, y=300)
        self.t6.place(x=220, y=300)
        self.lbl7.place(x=100, y=350)
        self.t7.place(x=220, y=350)
        self.lbl8.place(x=100, y=400)
        self.t8.place(x=220, y=400)
        self.lbl9.place(x=100, y=450)
        self.t9.place(x=220, y=450)
        self.lbl10.place(x=100, y=500)
        self.t10.place(x=220, y=500)
        self.lbl11.place(x=100, y=550)
        self.t11.place(x=220, y=550)
        self.lbl12.place(x=100, y=600)
        self.t12.place(x=220, y=600)
        self.lbl13.place(x=100, y=650)
        self.t13.place(x=220, y=650)
        self.lbl14.place(x=100, y=700)
        self.t14.place(x=220, y=700)
        self.lbl15.place(x=100, y=750)
        self.t15.place(x=220, y=750)
        self.b1 = Button(win, text='Send', command=self.command_pub)
        self.b1.place(x=100, y=800)
        self.pub = pub

    def command_pub(self):
        """ Publish commands on controller"""
        msg = GaitInput()
        msg.x = float(self.t1.get())
        msg.y = float(self.t2.get())
        msg.z = float(self.t3.get())
        msg.roll = float(self.t4.get())
        msg.pitch = float(self.t5.get())
        msg.yaw = float(self.t6.get())
        msg.StepLength = float(self.t7.get())
        msg.LateralFraction = float(self.t8.get())
        msg.YawRate = float(self.t9.get())
        msg.StepVelocity = float(self.t10.get())
        msg.ClearanceHeight = float(self.t11.get())
        msg.PenetrationDepth = float(self.t12.get())
        msg.SwingPeriod = float(self.t13.get())
        msg.YawControl = float(self.t14.get())
        msg.YawControlOn = float(self.t15.get())
        self.pub.publish(msg)


def main():
    spot_name = rospy.get_param('~/spot_name')
    rospy.init_node(spot_name + '_inverse_gui', anonymous=True)
    pub = rospy.Publisher('/' + spot_name + '/inverse_gait_input', GaitInput, queue_size=10)

    root = Tk()
    my_gui = RsGui(root, pub)
    root.title(spot_name + 'Control Input')
    root.geometry("400x800+10+10")
    root.mainloop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
