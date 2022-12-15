"""
This is a simple example of PD controller to control the torque of the single leg.
Using RMD v2.0 to set the maximum current to 60. otherwise it is very dangerous if there is something wrong in the code. 
--Yixiong

"""



import numpy as np
import serial
import time
import binascii
import struct
import re
import checksum_calculator as csum


class Motor:

    def __init__(self, id: str, serial, offset_torque: int=0, kp=1.5, kd=0) -> None:
        """
        offset_torque should be a integer value between -2000 ~ 2000
        """
        self.id = id
        self.ser = serial
        self.kp = kp
        self.kd = kd
        self.position_ref = self.read_position()
        self.i = 0
        self.torque_t = "0000"
        self.q_t = []
        self.tspan = []
        self.t_start = 0
        self.offset_torque = offset_torque

    def data_processing_position(self, data):
        '''
        receive the data from motor and read the encoder postion and return the angle in degree
        '''
        if data[0:2]!='3e' or data[2:4]!='92':
            return
        encoder_data=data[10:26]
        if encoder_data[-1]=='f':
            neghex_pack=binascii.unhexlify(encoder_data)
            neg=struct.unpack('q', neghex_pack)

            encoder_dec=neg[0]
        else:
            encoder_data=re.findall(r'\w{1,2}',encoder_data) #divide the data to groupds, '3e92' to ['3e','92']
            encoder_data=encoder_data[::-1]
            encoder_data = ''.join(encoder_data)
            encoder_dec=int(encoder_data,16)

        return encoder_dec/100/9

    def data_processing_torque(self, data):
        '''
        receive the data from motor and return the torque
        '''
        torque_data=data[12:16]
        torque_data=re.findall(r'\w{1,2}',torque_data)
        torque_data=torque_data[::-1]
        torque_data = ''.join(torque_data)
        torque_dec=int(torque_data,16)
        if torque_dec>2048: #if the torque is negetive
            torque_dec=torque_dec-65535-1
        
        return torque_dec

    def data_processing_speed(self, data):
        speed_data=data[16:20]
        speed_data=re.findall(r'\w{1,2}',speed_data)
        speed_data=speed_data[::-1]
        speed_data = ''.join(speed_data)
        speed_dec=int(speed_data,16)
        # print(speed_dec)
        if speed_dec>30000: #if the speed is negetive, the hex should be counted from the end
            speed_dec=speed_dec-65535

        return speed_dec/9

    def read_position(self):
        data = '3E 92 '+self.id+' 00'
        send_position=bytes.fromhex(data + " " + csum.compute_checksum8_mod256(data))
        self.ser.write(send_position)
        time.sleep(0.01)
        count=self.ser.inWaiting()
        if count>0:
            data=self.ser.read(count)
            data=binascii.b2a_hex(data).decode('gbk')
            # print(data)
            return self.data_processing_position(data)
        
    def read_motor_state_2(self):
        data = "3E 9C " + self.id + " 00"
        command = data + " " + csum.compute_checksum8_mod256(data)
        self.ser.write(bytes.fromhex(command))
        
        while(not self.ser.in_waiting()):
            pass
        data = self.ser.read(self.ser.in_waiting())



    def PD_controller(self,q_d,q_t,q_dot_t):
        '''
        here the kp and kd is the stiffness and damping. User defined
        based on the position error to calculate the torque
        '''
        torque=(self.kp*(q_d-q_t)-self.kd*q_dot_t)
        torque += self.offset_torque
        if torque < 0:
            torque=65535+torque
        torque=hex(int(torque))[2:6]
        template=list('0000')
        
        for i in range(len(torque)):
            template[-1-i]=torque[-1-i]
        
        template=''.join(template)
        return template

    def char_checksum(self, data, byteorder='little'):
        '''
        checksum function
        '''
        length = len(data)
        checksum = 0
        for i in range(0, length):
            x = int.from_bytes(data[i:i+1], byteorder, signed=True)
            if x>0 and checksum >0:
                checksum += x
                if checksum > 0x7F: 
                    checksum = (checksum&0x7F) - 0x80 
            elif x<0 and checksum <0:
                checksum += x
                if checksum < -0x80: 
                    checksum &= 0x7F
            else:
                checksum +=x 
            #print(checksum)    

        if checksum<0:
            checksum=256+checksum
        return hex(checksum)


    def torque_control(self, torque):
        '''
        based on the position error
        '''
        data1 = '3E A1 '+self.id+' 02'
        torque_command=list(data1 + " " + csum.compute_checksum8_mod256(data1) + ' E1 FF E0') #last part of string is placeholder
        torque_command[15]=torque[2]
        torque_command[16]=torque[3]
        torque_command[18]=torque[0]
        torque_command[19]=torque[1]
        if torque=='0000':
            cs='00'
        else:
            cs=self.char_checksum(bytes.fromhex(torque))[2:4]
            if len(cs)==1:
                cs='0'+cs[0]
        torque_command[-1]=cs[1]
        torque_command[-2]=cs[0]

        torque_command=''.join(torque_command)
        send_torque=bytes.fromhex(torque_command)
        self.ser.write(send_torque)
        time.sleep(0.01)
        count=self.ser.inWaiting()
        if count>0:
            data=self.ser.read(count)
            data=binascii.b2a_hex(data).decode('gbk')
            # print(data)
            # print(data_processing_torque(data))

    def read_torque(self):
        '''
        read torque and speed
        '''
        data = '3E 9C ' + self.id +' 00 '
        command_torque= data+csum.compute_checksum8_mod256(data)
        send_torque=bytes.fromhex(command_torque)
        self.ser.write(send_torque)
        time.sleep(0.01)
        count=self.ser.inWaiting()
        if count>0:
            data=self.ser.read(count)
            data=binascii.b2a_hex(data).decode('gbk')
            return self.data_processing_speed(data)
            # print(data)
            # print(data_processing_torque(data))

    def Step(self, print=False):
            
            if not self.position_ref:
                self.position_ref = self.read_position()
            q_t=self.read_position()
            self.q_t.append(q_t-self.position_ref)
            self.tspan.append(time.perf_counter()-self.t_start)

            if (5<self.i and print):
                print("position ", q_t)
                
                print("torque ", int(torque_t.lstrip("0") or "0", 16))
                self.i=0
            q_dot_t=self.read_torque()
            torque_t=self.PD_controller(self.position_ref, q_t, q_dot_t) # set kd to zero first. because it is dangerous. 

            self.i+=1
            self.torque_control(torque_t)
    
    def start(self):
        self.tstart = time.perf_counter()

    def stop(self):
        data = "3E 81 " + self.id + " 00"
        command = data + " " +csum.compute_checksum8_mod256(data)
        self.ser.write(bytes.fromhex(command))
        self.savetxt()
        self.q_t = []
        self.tspan = []

    def savetxt(self):
        np.savetxt("position_data_"+self.id, np.array([self.tspan, self.q_t]))


if __name__ == "__main__":
    from ast import literal_eval
    ser1=serial.Serial("COM6",115200)
    ser2=serial.Serial("COM5",115200)
    m1 = Motor("05", ser2, kp=4, offset_torque=0)
    m2 = Motor("06", ser1, kp=1, offset_torque=0)
    print("Move the robot to desired offset, then press enter")
    input()
    m1.start()
    m2.start()
    try:
        while True:
            start = time.perf_counter()
            m1.Step()
            elapsed1 = time.perf_counter() - start 
            m2.Step()
            time.sleep(0.0001)
            elapsed = time.perf_counter()-start   
    except KeyboardInterrupt:
        print(elapsed1)
        print(elapsed)
        m1.stop()
        m2.stop()
        m1.ser.close()
        m2.ser.close()
# print(char_checksum(bytes.fromhex('ECFF')))
