import serial
import struct


######################################################################################################################
def readlineSerial(port):
    new_bytes = bytearray()
    start_byte = bytearray()
    start_byte = b'\xc0'                    #SLIP END byte
    start2_byte = bytearray()
    start2_byte = b'\x01'                   #maybe buffer byte??

    svgsStateMsg = bytearray()

    vectors = []

    msgStart = 0
    msgEnd = 4

    new_bytes = port.read()
    #print(new_bytes)
    
    if(new_bytes == start_byte):        #checks for slip start
        #print("start byte found")
        new_bytes = port.read()
        #print(new_bytes)
        if(new_bytes == start2_byte):   #checks for the other start byte
            #print("2nd start byte found")
            #message is begining
            #Will loop til message ends
            while True:        #while the last byte is not the end byte
                i=0
                
                for i in range(4):
                    new_bytes = port.read()     #takes in new bytes into a temp variable


                    #return condition, not good coding practice but works
                    if(new_bytes == start_byte):
                        return vectors
                        
                    svgsStateMsg += new_bytes   #adds temp variable bytes to svgs State Msg
                    i+=1
                
                temp = struct.unpack('f', svgsStateMsg[msgStart:msgEnd])
                msgStart += 4
                msgEnd += 4

                vectors += temp
                #print(vectors)
                
                
########################################################################################################################
    
#because of while loop, timeout is how long lock of ttyS0 will be if an error occures
port = serial.Serial("/dev/ttyAMA0", baudrate = 57600, timeout = 1.0)     
vectors = []                                                            

while True:
    vectors = readlineSerial(port)

    if(vectors != None):
        #post to ROS here
        print("X:       %1.5f" % vectors[0])
        print("Y:       %1.5f" % vectors[1])
        print("Z:       %1.5f" % vectors[2])
        print("R:       %1.5f" % vectors[3])
        print("P:       %1.5f" % vectors[4])
        print("Theta_Y: %1.5f\n" % vectors[5])
    else:
        print("Data not recieved")

