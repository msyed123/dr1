import serial
import struct

print("Started\n")

class slip:
        byteMsg     = bytearray()
        END         = b'\xc0'   #192
        ESC         = b'\xdb'   #219
        ESC_END     = b'\xdc'   #220
        ESC_ESC     = b'\xdd'   #221

        started     = False
        escaped     = False

        vectors     =[]

        def toVector(self):
            try:
                for i in range(7):
                    self.vectors += struct.unpack('f', self.byteMsg[1+(i*4):1+((i+1)*4)])   #byteMsg[0] is svgs mode
            except:
                Exception("PACKET FAILED")

        def zeroCheck(self):
            if(SLIP.vectors[0] == 0.0 and SLIP.vectors[1] == 0.0 and SLIP.vectors[2] == 0.0):
                return(False)
            else:
                return(True)
                

        def readSerial(self,port):
            new_byte = bytearray()
            try:
                new_byte = port.read()
            except:
                print("could not read")
                raise Exception("COULD NOT READ")
                return(False)
                
            #END
            if(new_byte == self.END):
                if self.started:
                    return(True)
                else:
                    self.started = True

            #ESC
            elif new_byte == self.ESC:
                self.escaped = True         
                
            #ESC_END
            elif new_byte == self.ESC_END:
                if self.escaped:
                    self.byteMsg += self.END
                    self.escaped = False
                else:
                    self.byteMsg += new_byte
                    
            #ESC_ESC
            elif new_byte == self.ESC_ESC:
                if self.escaped:
                    self.byteMsg += self.ESC
                    self.escaped = False
                else:
                    self.byteMsg += new_byte

            #ALL OTHERS
            else:
                if(self.escaped):
                    raise Exception('Slip Protocol Error')
                    self.byteMsg = ''
                    self.escaped = False
                else:
                    if(self.started):
                        self.byteMsg += new_byte
                    else:
                        Exception('COMM ERROR')
            
                        
                    
########################################################################################################################


#because of while loop, timeout is how long lock of ttyS0 will be if an error occures
#original baudrate = 57600
port = serial.Serial("/dev/ttyAMA0", baudrate = 38400, timeout = 1.0)


msgValid = False
SLIP = slip()

while True: 
    msgValid = SLIP.readSerial(port)
    
    if(msgValid is True and len(SLIP.byteMsg)>1):
        print("valid message recieved")
        SLIP.toVector()
	#print(SLIP.vectors)
        msgValid = False       
        if(SLIP.zeroCheck()):
            #IF(DATA_FILTER()):
                #post to ROS here
            print("X:       %1.5f" % SLIP.vectors[0])
            print("Y:       %1.5f" % SLIP.vectors[1])
            print("Z:       %1.5f" % SLIP.vectors[2])
            print("R:       %1.5f" % SLIP.vectors[3])
            print("P:       %1.5f" % SLIP.vectors[4])
            print("Theta_Y: %1.5f\n" % SLIP.vectors[5])

        #CLEAN UP
        SLIP.byteMsg = b''
        SLIP.vectors = []
        
        
    
