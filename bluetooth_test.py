import bluetooth


def connect():
    while(True):    
        try:
            gaugeSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            gaugeSocket.connect(('00:A0:96:37:4A:3F', 1))
            break;
        except bluetooth.btcommon.BluetoothError as error:
            gaugeSocket.close()
            print "Could not connect: ", error, "; Retrying in 10s..."
    return gaugeSocket;

gaugeSocket = connect()
while(True):

    try:
        data = gaugeSocket.recv(1024)
	print 'recv: {0:d} bytes'.format(len(data))
        for i, char in enumerate(data):
	    print 'byte: {0:d} value: 0x{1:x}'.format( i, ord(char))
    except bluetooth.btcommon.BluetoothError as error:
        print "Caught BluetoothError: ", error
        time.sleep(5)
        gaugeSocket = connect()
        pass

gaugeSocket.close()
