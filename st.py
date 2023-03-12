import serial as s
import shlex
import time

# Use this one for Mac/Linux
# DEFAULT_DEV = '/dev/tty.KeySerial1'

# Use this one for PC
DEFAULT_DEV = 'COM7'
# DEFAULT_BAUD_RATE = 9600
DEFAULT_BAUD_RATE = 19200
DEFAULT_TIMEOUT = 0.1

# Roboforth Strings
CR = b'\r'
LF = b'\n'

PURGE = b'PURGE'
ROBOFORTH = b'ROBOFORTH'
DECIMAL = b'DECIMAL'
START = b'START'
JOINT = b'JOINT'
CALIBRATE = b'CALIBRATE'
HOME = b'HOME'
WHERE = b'WHERE'
CARTESIAN = b'CARTESIAN'
SPEED = b'SPEED'
ACCEL = b'ACCEL'
MOVETO = b'MOVETO'
HAND = b'HAND'
WRIST = b'WRIST'
ENERGIZE = b'ENERGIZE'
DE_ENERGIZE = b'DE-ENERGIZE'
QUERY = b' ?'
IMPERATIVE = b' !'
TELL = b'TELL'
MOVE = b'MOVE'
CARTESIAN_NEW_ROUTE = b'CARTESIAN NEW ROUTE'
RESERVE = b'RESERVE'

OK = b'OK'


class StPosCart():

    def __init__(self, pos=[0, 0, 0, 0, 0]):
        self.set(pos)

    def set(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.pitch = pos[3]
        self.roll = pos[4]

    def __repr__(self):
        return 'X=%smm, Y=%smm, Z=%smm, Pitch=%sdeg, Roll=%sdeg' % (self.x/10.0,
                                                         self.y/10.0,
                                                         self.z/10.0,
                                                         self.pitch/10.0,
                                                         self.roll/10.0)


class StArm():
    '''Class for controlling the 5-axis R17 arm from ST Robotics'''

    '''
    Description:
    Create a serial connection and open it.

    Inputs:
        dev_name: The name of the serial device. For Macs/Linux, use
        /dev/tty.somestringofcharsandnums and for PCs use COMX where
        X is the COM port number the serial connector for the arm is
In [36]: 
        connected to.
    '''

    def __init__(self, dev=DEFAULT_DEV, baud=DEFAULT_BAUD_RATE,
                 startup=True, to=DEFAULT_TIMEOUT, debug=False):
        self.cxn = s.Serial(dev, baudrate=baud, timeout=to)
        # TODO
        # Check and parse return values of all ROBOFORTH methods called.
        self.debug = debug
        self.curr_pos = StPosCart()
        self.prev_pos = StPosCart()
        self.block_timeout = 10
        if startup:
            try:
                self.startup()
            except:
                print("startup procedure failed")
        self.tool_length = 0

    def startup(self):
        self.cxn.flushInput()
        self.purge()
        self.roboforth()
        self.start() 
        
        self.calibrate()           
        self.home()
        self.joint()
        self.where()


    def set_tool_length(self, length):
        self.tool_length = length

    def purge(self):
        cmd = PURGE
        print('Purging...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def roboforth(self):
        cmd = ROBOFORTH
        print('Starting RoboForth...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def decimal(self):
        print('Setting decimal mode...')
        self.cxn.flushInput()
        self.cxn.write(DECIMAL + CR)

    def start(self):
        cmd = START
        print('Starting...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def joint(self):
        cmd = JOINT
        print('Setting Joint mode...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def calibrate(self):
        cmd = CALIBRATE
        print('Calibrating...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def home(self):
        cmd = HOME
        print('Homing...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def cartesian(self):
        cmd = CARTESIAN
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def block_on_result(self, cmd):
        t = time.time()
        s = self.cxn.read(self.cxn.inWaiting())
        
        while s[-5:-3] != OK:
            # print(s)
            #Match '>' only at the end of the string
            start_time = time.time()
            if s[-1:] == b'>':
                if self.debug:
                    print('Command ' + str(cmd) + ' completed without ' +
                          'verification of success.')
                raise Exception('Arm command failed to execute as expected.', s)
            s += self.cxn.read(self.cxn.inWaiting())
            if time.time() - start_time > self.block_timeout:
                raise Exception('block on result timed out', s)

        if self.debug:
            print('Command ' + str(cmd) + ' completed successfully.')
        return str(s,encoding="utf-8")

    def get_status(self):
        if self.cxn.isOpen():
            self.cxn.write('' + CR)

    def get_speed(self):
        cmd = SPEED + QUERY
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        result = self.block_on_result(cmd)
        return int(result.split(' ')[-2])

    def set_speed(self, speed):
        cmd = bytes(str(speed),"utf-8") + b' ' + SPEED + IMPERATIVE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def get_accel(self):
        cmd = ACCEL + QUERY
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        result = self.block_on_result(cmd)
        return int(result.split(' ')[-2])

    def set_accel(self, accel):
        cmd = bytes(str(accel),"utf-8") + b' ' + ACCEL + IMPERATIVE
        print('Setting acceleration to %d' % accel)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def move_to(self, pos, block=True):
        self.cartesian()
        cmd = str(pos[0]) + ' ' + str(pos[1]) + ' ' + str(pos[2]) +  ' MOVETO'
        cmd = bytes(cmd,"utf-8")
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        if block:
            self.block_on_result(cmd)
            self.where()

    def joint_move(self,pos,block=True):
        """
        move to joint position [waist,shoulder,elbow,hand,wrist] 
        Note: units are in whatever ST robotics use, I recommend choosing poses by trial and error
        """

        cmd = bytes(str(pos[4]),"utf-8") +b" "+ bytes(str(pos[3]),"utf-8") +b" "+ bytes(str(pos[2]),"utf-8") +b" "+ bytes(str(pos[1]),"utf-8") +b' '+ bytes(str(pos[0]),"utf-8") +b' JMA'
        
        self.joint()
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.where()

    def move(self, del_pos):
        cmd = bytes(str(del_pos[0]),"utf-8")  + b' ' + bytes(str(del_pos[1]),"utf-8")  + b' ' + bytes(str(del_pos[2]),"utf-8")  + b' MOVE'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.where()

    def rotate_wrist(self, roll):
        cmd = TELL + b' ' + WRIST + b' ' + bytes(str(roll),"utf-8") + b' ' + MOVETO
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def rotate_wrist_rel(self, roll_inc):
        cmd = TELL + b' ' + WRIST + b' ' + bytes(str(roll_inc),"utf-8") + b' ' + MOVE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.cartesian()
        self.where()

    def rotate_hand(self, pitch):
        cmd = TELL + b' ' + HAND + b' ' + bytes(str(pitch),"utf-8") + b' ' + MOVETO
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        # self.cartesian()
        # self.where()

    def rotate_hand_rel(self, pitch_inc):
        cmd = TELL + b' ' + HAND + b' ' + bytes(str(pitch_inc), "utf-8") + b' ' + MOVE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.cartesian()
        self.where()

    def energize(self):
        cmd = ENERGIZE
        print('Powering motors...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def de_energize(self):
        cmd = DE_ENERGIZE
        print('Powering down motors...')
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def where(self):
        self.cartesian()
        cmd = WHERE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        res = self.block_on_result(cmd)
#         try:
        print(res)
        lines = res.split('\r\n')
            #TODO: Need to account for possibility that arm is in decimal mode

        cp = [int(x.strip().replace('.', '')) for x in shlex.split(lines[2])]
        pp = [int(x.strip().replace('.', '')) for x in shlex.split(lines[3])[1:]]
        self.curr_pos.set(cp)
        self.prev_pos.set(pp)
#         except RuntimeError, e:
#             print('Exception in where.')
#             print(e)
#             self.curr_pos.set([0, 0, 0, 0, 0])
#             self.prev_pos.set([0, 0, 0, 0, 0])

        return (self.curr_pos, self.prev_pos)

    def close_connection(self):
        self.cxn.close()
        print("closed the serial connection")
