from .bno055 import BNO055
from .utils import *
import time
import json

class StrapDown():
    def __init__(self):
        self._imu = BNO055()
        self._imu.initialize()
        status, self_test, error = self._imu.get_system_status() #IDK why but you need this
        self._attitude = Quaternion()
        self._omega = Vector()
        self._velocity = Vector()
        self._position = Vector()
        self._gyro = Vector()
        self._accl = Vector()
        self.prev_time = time.time()
        self._start_time = time.time()
        self._events = []
        self._gravity = None
        self._loop()
    def _loop(self):
        while True:
            self._thread_function()
            self._position.print()
            event = json.dumps({'time_stamp':self.prev_time,
                'atitude':self._attitude.json(),
                'position':self._position.json(),
                'velocity':self._velocity.json(),
                'gyro':self._gyro.json(),
                'imu':self._accl.json(),
                'omega':self._omega.json()})
            self._events.append(event)
    
    def _thread_function(self):
        #Time Step
        curr_time = time.time()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time

        #Data Collection
        wx,wy,wz = self._imu.read_gyroscope()
        ax,ay,az = self._imu.read_accelerometer()
        #sanity Check values
        if abs(wx) > 40 or  abs(wy) > 40 or abs(wz) > 40:
            return
        if abs(ax) > 100 or  abs(ay) > 100 or abs(az) > 100:
            return
        alpha = Vector(0,wx,wy,wz)
        a_b = Vector(0,ax,ay,az)
        self._gyro = alpha
        self._accl = a_b

        #Add Gravity if needed
        if self._gravity == None:
            self._gravity = a_b

        #Integrate Gyroscope to update Attitude
        omega = alpha.integrate(dt)
        self._omega = self._omega.plus(omega).unit()
        theta = self._omega.integrate(dt)
        self._attitude = self._attitude.plus(theta).unit()

        #Subtract out gravity, convert acceleration to Inertial Frame and Integrate to update position
        a_i = self._attitude.hamilton(a_b).hamilton(self._attitude.conjugate())
        print("HERE")
        a_i_g = a_i.gravity(self._attitude,self._gravity)
        v_i = a_i_g.integrate(dt)
        self._velocity = self._velocity.plus(v_i)
        position = self._velocity.integrate(dt)
        position._a = 0
        self._position = self._position.plus(position)
    
    
    def __del__(self):
        file_name = "accl_" + str(self._start_time) + ".json"
        with open(file_name, 'w') as f:
            for event in self._events:
                f.write(event + '\n')