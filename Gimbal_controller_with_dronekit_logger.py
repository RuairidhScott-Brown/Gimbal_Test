
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import math
import pandas as pd
import threading
import os
import subprocess
import signal



moment = time.strftime("%Y-%b-%d__%H-%M-%S", time.localtime())
startTime = time.time()
condition = threading.Condition()
queue = []
exit =[]
bad_pitch = -110

def connecting(con):
    if con == 2:
        print("Try something else, the connection isnt working")

    else:
        try:
            vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200, heartbeat_timeout=300)
        except(NameError):
            print("need to allow permission")
            subprocess.call("sudo chmod 666 /dev/ttyACM0", shell=True)
            time.sleep(0.5)
            print("changed permission")
            return connecting(con + 1)
        else:
            print(vehicle)
            return vehicle

def file():
    global bad_pitch
    if not vehicle.gimbal.pitch < bad_pitch:
        print("filing")
        pro = subprocess.Popen("xterm -e 'tail -f /home/livelink/Documents/Gimbal/gimbal_log/visual_logger.txt'", stdout=subprocess.PIPE,
                               shell=True, preexec_fn=os.setsid)
        while True:
            if vehicle.gimbal.pitch < bad_pitch:
                os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
                print("broken")
                break





class Gimbal_control(threading.Thread):

    def __init__(self):
        super(Gimbal_control, self).__init__()
        self._stopevent = threading.Event()


    def run(self):
        print("Logging Running")
        gimbalAttitude = pd.DataFrame(columns=['Time','Roll', 'Pitch', 'Yaw'])
        while not self._stopevent.isSet():
                with open("visual_logger.txt", "w") as f:
                    time.sleep(0.5)
                    f.write("pith is: " +str(vehicle.gimbal.pitch - 25))
                gimbalAttitude = gimbalAttitude.append({'Time': time.time() , 'Roll': vehicle.gimbal.roll , 'Pitch': vehicle.gimbal.pitch , 'Yaw': vehicle.gimbal.yaw}, ignore_index=True)
                time.sleep(0.5)
                #print("pitch is: " + str(vehicle.gimbal.pitch - 25))
        vehicle.gimbal.rotate(-30,0,0)
        time.sleep(7)
        print("logging stopped")
        gimbalAttitude['Time Difference'] = gimbalAttitude['Time']-startTime
        gimbalAttitude.to_csv("/home/livelink/Documents/Gimbal/gimbal_log/Logs/output" + moment + ".csv", sep=',', mode='a')

    def end(self):
        global bad_pitch
        print("ending good")
        while not self._stopevent.isSet():
            time.sleep(0.5)
            if vehicle.gimbal.pitch < bad_pitch:
                print("ending")
                time.sleep(0.5)
                self._stopevent.set()
        print("ended")

class Pitch_control(threading.Thread):
    def __init__(self):
        super(Pitch_control, self).__init__()
        self._stopevent = threading.Event()

    def run(self):
        while not self._stopevent.isSet():
            global queue
            global bad_pitch
            condition.acquire()
            if not queue:
                condition.wait()
            pitch = queue.pop(0)
            vehicle.gimbal.rotate(int(pitch),0,0)
            condition.release()
            time.sleep(0.5)
        print("edning_p")

    def end(self):
        global bad_pitch
        print("ending good p")
        while not self._stopevent.isSet():
            time.sleep(0.5)
            if vehicle.gimbal.pitch < bad_pitch:
                print("ending p")
                self._stopevent.set()
        print("ended_pitch p")


class demmand(threading.Thread):
    def __init__(self):
        super(demmand, self).__init__()
        self._stopevent =threading.Event()

    def run(self):
        while not self._stopevent.isSet():
            global queue
            global bad_pitch
            condition.acquire()
            dem = input("pitch: ")
            queue.append(int(dem))
            condition.notify()
            condition.release()
            time.sleep(0.5)
        print("ending d")

    def end(self):
        global bad_pitch
        print("ending good d")
        while not self._stopevent.isSet():
            time.sleep(0.5)
            if vehicle.gimbal.pitch < bad_pitch:
                print("ending d")
                self._stopevent.set()
        print("ended_demmand d")




if __name__ == "__main__":
    vehicle = connecting(0)

    thread_list = []
    thread_file_list = []
    thread_p_list = []
    thread_d_list = []

    vehicle.gimbal.rotate(-30,0,0)

    thread_file = threading.Thread(target=file)
    thread = Gimbal_control()
    thread_p = Pitch_control()
    thread_d = demmand()

    thread_file_list.append(thread_file)
    thread_list.append(thread)
    thread_p_list.append(thread_p)
    thread_d_list.append(thread_d)

    thread_file.setDaemon(True)
    thread_p.daemon = True
    thread_d.daemon = True

    thread.start()
    pro = thread_file.start()
    thread_p.start()
    thread_d.start()

    theEnd = thread.end()
    thread_p.end()
    thread_d.end()



    if thread.isAlive() == False:
        print("main thread is dead")
        print(thread_list)
        print("Terminal shoud be gone")

    if thread_p.isAlive() == False:
        print("the consumer is dead")
    if thread_d.isAlive() == False:
        print("the producer is dead")
    if thread.isAlive() == True:
        print("main thread is alive")
    if thread_p.isAlive() == True:
        print("the consumer is Alive")
        print(thread_p_list)
    if thread_d.isAlive() == True:
        print("the producer is Alive")
        print(thread_d_list)
