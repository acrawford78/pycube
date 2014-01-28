#!/usr/bin/python
"""

eMPL-wSDK-client.py

Copyright 2011 InvenSense, Inc. All Rights Reserved.


"""
import serial, sys, time, string, pygame, bluetooth
from drawcube import *
class PacketReader(object):
    def __init__(self, quatdelegate=None, debugdelegate=None, datadelegate=None):
        print "packet reader"
        if quatdelegate:
            self.quatdelegate = quatdelegate
        else:
            self.quatdelegate = emptyPacketDelegate()

        if debugdelegate:
            self.debugdelegate = debugdelegate
        else:
            self.debugdelegate = emptyPacketDelegate()

        if datadelegate:
            self.datadelegate = datadelegate
        else:
            self.datadelegate = emptyPacketDelegate()
    
    def process_packet(self, buf):
        pktcode = buf[1]
        print "handler for pktcode",pktcode
        if pktcode == 1:
            d = debugPacket(buf)
            self.debugdelegate.dispatch(d)
        elif pktcode == 2:
            p = quatPacket(buf)
            self.quatdelegate.dispatch(p)
        elif pktcode == 3:
            d = dataPacket(buf)
            self.datadelegate.dispatch(d)
        else:
            print "no handler for pktcode",pktcode
            pktcode = -2

        return pktcode
    
class SerialPacketReader(PacketReader):
    def __init__(self, port, quatdelegate=None, debugdelegate=None, datadelegate=None):
        #call super class
        super(SerialPacketReader, self).__init__(quatdelegate, debugdelegate, datadelegate)        
        self.s = serial.Serial(port,115200)
        self.s.setTimeout(1.0)
        self.s.setWriteTimeout(0.2)

        self.packets = []
        self.length = 0
        self.previous = None

    def read(self):
        NUM_BYTES = 14
        p = None
        while self.s.inWaiting() >= NUM_BYTES:
            #read entire buffer
            rs = self.s.read(NUM_BYTES)
            if ord(rs[0]) == ord('$'):
                pktcode = ord(rs[1])
                if pktcode == 1:
                    d = debugPacket(rs)
                    self.debugdelegate.dispatch(d)
                elif pktcode == 2:
                    p = quatPacket(rs)
                    self.quatdelegate.dispatch(p)
                elif pktcode == 3:
                    d = dataPacket(rs)
                    self.datadelegate.dispatch(d)
                else:
                    print "no handler for pktcode",pktcode
            else:
                c = ' '
                print "serial misaligned!"
                while not ord(c) == ord('$'):
                    c = self.s.read(1)
                self.s.read(NUM_BYTES-1)
    
    def write(self,a):
        self.s.write(a)

    def close(self):
        self.s.close()

    def write_log(self,fname):
        f = open(fname,'w')
        for p in self.packets:
            f.write(p.logfile_line())
        f.close()

class BluetoothPacketReader(PacketReader):
    def __init__(self, bt_addr, quatdelegate=None, debugdelegate=None, datadelegate=None ):
        #call super class
        super(BluetoothPacketReader, self).__init__(quatdelegate, debugdelegate, datadelegate)       
        self.bt_addr = bt_addr
    def connect(self):
        connected = False  
        try:
            self.bt_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            #self.bt_socket.settimeout(1.0)
            self.bt_socket.connect((self.bt_addr, 1))
            #self.bt_socket.setblocking(False)
            connected = True    
        except bluetooth.btcommon.BluetoothError as error:
            self.bt_socket.close()
            print "Could not connect: ", error
        return connected;

    def read(self):
        NUM_BYTES = 14
        try:
            #non blocking recv
            packet_complete = False
            data = self.bt_socket.recv(NUM_BYTES)
            print "recv: ", len(data)
            pkt = bytearray()
            if ord(data[0]) != ord('$'):
                print "sync missing"    
            for i, char in enumerate(data):
                if ord(char) == ord('$'):
                    pkt = bytearray(data[i:len(data)])
                    break
            #mop up remaning bytes 
            while len(pkt) < NUM_BYTES:
                bytes_left = NUM_BYTES - len(pkt)
                pkt.extend(self.bt_socket.recv(bytes_left))

            ret = super(BluetoothPacketReader, self).process_packet(pkt)
            if ret < 0:
                print "no handler for pktcode", ret
        
        except bluetooth.btcommon.BluetoothError as error:
            print "Caught BluetoothError: ", error
            pass
   
#===========  PACKET DELEGATES  ==========

class packetDelegate(object):
    def loop(self,event):
        print "generic packetDelegate loop w/event",event
    def dispatch(self,p):
        print "generic packetDelegate dispatched",p

class emptyPacketDelegate(packetDelegate):
    def loop(self,event):
        pass
    def dispatch(self,p):
        pass

class cubePacketViewer (packetDelegate):
    def __init__(self):
        self.screen = Screen(480,400,scale=1.5)
        self.cube = Cube(60,60,60)
        self.q = Quaternion(1,0,0,0)
        self.previous = None  # previous quaternion
        self.latest = None    # latest packet (get in dispatch, use in loop)

    def loop(self,event):
        packet = self.latest
        if packet:
            q = packet.to_q().normalized()
            self.cube.erase(self.screen)
            self.cube.draw(self.screen,q)
            pygame.display.flip()
            self.latest = None

    def dispatch(self,p):
        if isinstance(p,quatPacket):
            self.latest = p

class debugPacketViewer (packetDelegate):
    def loop(self,event):
        pass
    def dispatch(self,p):
        assert isinstance(p,debugPacket);
        p.display()

class dataPacketViewer (packetDelegate):
    def loop(self,event):
        pass
    def dispatch(self,p):
        assert isinstance(p,dataPacket);
        p.display()

# =============== PACKETS =================

# utility function:
def twoBytes(d1,d2):
    """ unmarshal two bytes into int16 """
    d = (d1)*256 + (d2)
    if d > 32767:
        d -= 65536
    return d

class debugPacket (object):
    """ body of packet is a debug string """
    def __init__(self,l):
        sss = []
        for c in l[2:12]:
            if ord(c) != 0:
                sss.append(c)
        self.s = "".join(sss)

    def display(self):
        sys.stdout.write(self.s)

class dataPacket (object):
    def __init__(self, l):
        self.data = [0,0,0,0]
        self.type = l[10]
        # each data component is transmitted as a signed 16 byte int
        if l[10] == 'g':      # gyro, q5
            div = 1.0 / (1 << 5)
            self.num_elements = 3
        elif l[10] == 'a':    # accel, q12
            div = 1.0 / (1 << 12)
            self.num_elements = 3
        elif l[10] == 'm':    # compass, q8
            div = 1.0 / (1 << 8)
            self.num_elements = 3
        elif l[10] == 'q':    # quaternion, q14
            div = 1.0 / (1 << 14)
            self.num_elements = 4
        elif l[10] == 's':    # temp slope, q8
            div = 1.0 / (1 << 8)
            self.num_elements = 3
        elif l[10] == 'h':    # heading, q0
            div = 1.0
            self.num_elements = 1
        elif l[10] == 'x':    # accel bias, q12
            div = 1.0 / (1 << 12)
            self.num_elements = 3
        elif l[10] == 'b':    # gyro bias, q5
            div = 1.0 / (1 << 5)
            self.num_elements = 3
        elif l[10] == 'c':    # compass bias, q8
            div = 1.0 / (1 << 8)
            self.num_elements = 3
        else:                 # not yet supported
            div = 1

        for ii in range(0,self.num_elements):
            self.data[ii] = twoBytes(l[2*ii+2],l[2*ii+3]) * div

        # packet count
        self.counter = ord(l[11])

    def display(self):
        if self.type == 'g':
            print 'gyro: %7.3f %7.3f %7.3f' % \
                (self.data[0], self.data[1], self.data[2])
        elif self.type == 'a':
            print 'accel: %9.5f %9.5f %9.5f' % \
                (self.data[0], self.data[1], self.data[2])
        elif self.type == 'm':
            print 'mag: %7.4f %7.4f %7.4f' % \
                (self.data[0], self.data[1], self.data[2])
        elif self.type == 'q':
            print 'quat: %7.4f %7.4f %7.4f %7.4f' % \
                (self.data[0], self.data[1], self.data[2], self.data[3])
        elif self.type == 's':
            print 'temp slope: %7.4f %7.4f %7.4f' % \
                (self.data[0], self.data[1], self.data[2])
        elif self.type == 'h':
            print 'heading: %7.4f' % self.data[0]
        elif self.type == 'b':
            print 'gyro biases: %7.3f %7.3f %7.3f' % \
                (self.data[0], self.data[1], self.data[2])
        elif self.type == 'x':
            print 'accel biases: %9.5f %9.5f %9.5f' % \
                (self.data[0], self.data[1], self.data[2])
        elif self.type == 'c':
            print 'mag biases: %7.4f %7.4f %7.4f' % \
                (self.data[0], self.data[1], self.data[2])
        else:
            print 'what?'

class quatPacket (object):
    def __init__(self, l):
        """ list of 8 bytes expected."""
        self.l = l
        # each quat component is transmitted as a signed 16 byte int
        # fixed point where 1 = 2^14 LSB
        div = 1.0 / (1 << 14)
        self.q0 = twoBytes(l[2],l[3]) * div
        self.q1 = twoBytes(l[4],l[5]) * div
        self.q2 = twoBytes(l[6],l[7]) * div
        self.q3 = twoBytes(l[8],l[9]) * div

        # packet counter is 8 bit unsigned int
        self.counter = l[11]

    def display_raw(self):
        l = self.l
        print "".join(
            [ str((l[0])), " "] + \
            [ str((l[1])), " "] + \
            [ str((a)).ljust(4) for a in
                                [ l[2], l[3], l[4], l[5], l[6], l[7], l[8], l[9], l[10] ] ] + \
            [ str((a)).ljust(4) for a in
                                [ l[8], l[9], l[10] , l[11], l[12], l[13]] ]
            )

    def display(self):
        if 1:
            print "qs " + " ".join([str(s).ljust(15) for s in
                [ self.q0, self.q1, self.q2, self.q3 ]])
        if 0:
            euler0, euler1, euler2 = self.to_q().get_euler()
            print "eulers " + " ".join([str(s).ljust(15) for s in
                [ euler0, euler1, euler2 ]])
        if 0:
            euler0, euler1, euler2 = self.to_q().get_euler()
            print "eulers " + " ".join([str(s).ljust(15) for s in
                [ (euler0 * 180.0 / 3.14159) - 90 ]])



    def to_q(self):
        return Quaternion(self.q0, self.q1, self.q2, self.q3)

# =============== MAIN ======================

if __name__ == "__main__":
    if len(sys.argv) == 2:
        comport = int(sys.argv[1]) - 1
		#if comport < 0:
			#use socket
    else:
        print "usage: " + sys.argv[0] + " port"
        sys.exit(-1)


    pygame.init()
    viewer = cubePacketViewer()
    debug  = debugPacketViewer()
    data   = dataPacketViewer()
    
    if(comport < 0):
        bt_addr = '00:A0:96:37:4A:3F'
        reader = BluetoothPacketReader(bt_addr,
                 quatdelegate = viewer,
                 debugdelegate = debug,
                 datadelegate = data)
        while reader.connect() == False:
            time.sleep(5)

    else:
        reader = emplPacketReader(comport,
                 quatdelegate = viewer,
                 debugdelegate = debug,
                 datadelegate = data)

    while 1:
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            viewer.close()
            break
        if event.type == pygame.KEYDOWN:
            reader.write(pygame.key.name(event.key))

        reader.read()
        viewer.loop(event)
        debug.loop(event)
        data.loop(event)

        pygame.time.wait(0) # if system load is too high, increase this 'sleep' time.
