"""
This is a simple example of PD controller to control the torque of the single leg.
Using RMD v2.0 to set the maximum current to 60. otherwise it is very dangerous if there is something wrong in the code. 
--Yixiong

"""



import serial
import time
import binascii
import struct
import re

def data_processing_position(data):
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

def data_processing_torque(data):
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

def data_processing_speed(data):
    speed_data=data[16:20]
    speed_data=re.findall(r'\w{1,2}',speed_data)
    speed_data=speed_data[::-1]
    speed_data = ''.join(speed_data)
    speed_dec=int(speed_data,16)
    # print(speed_dec)
    if speed_dec>30000: #if the speed is negetive, the hex should be counted from the end
        speed_dec=speed_dec-65535

    return speed_dec/9

def read_position():
    send_position=bytes.fromhex('3E 92 01 00 D1')
    ser.write(send_position)
    time.sleep(0.01)
    count=ser.inWaiting()
    if count>0:
        data=ser.read(count)
        data=binascii.b2a_hex(data).decode('gbk')
        # print(data)
        return data_processing_position(data)

def PD_controller(kp,kd,q_d,q_t,q_dot_t):
    '''
    here the kp and kd is the stiffness and damping. User defined
    based on the position error to calculate the torque
    '''
    torque=(kp*(q_d-q_t)-kd*q_dot_t)
    if torque < 0:
        torque=65535+torque
    torque=hex(int(torque))[2:6]
    template=list('0000')
    
    for i in range(len(torque)):
        template[-1-i]=torque[-1-i]
    
    template=''.join(template)
    return template

def char_checksum(data, byteorder='little'):
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


def torque_control(torque):
    '''
    based on the position error
    '''
    torque_command=list('3E A1 01 02 E2 E1 FF E0') #just template
    torque_command[15]=torque[2]
    torque_command[16]=torque[3]
    torque_command[18]=torque[0]
    torque_command[19]=torque[1]
    if torque=='0000':
        cs='00'
    else:
        cs=char_checksum(bytes.fromhex(torque))[2:4]
        if len(cs)==1:
            cs='0'+cs[0]
    torque_command[-1]=cs[1]
    torque_command[-2]=cs[0]

    torque_command=''.join(torque_command)
    send_torque=bytes.fromhex(torque_command)
    ser.write(send_torque)
    time.sleep(0.01)
    count=ser.inWaiting()
    if count>0:
        data=ser.read(count)
        data=binascii.b2a_hex(data).decode('gbk')
        # print(data)
        # print(data_processing_torque(data))

def read_torque():
    '''
    read torque and speed
    '''
    command_torque='3E 9C 01 00 DB'
    send_torque=bytes.fromhex(command_torque)
    ser.write(send_torque)
    time.sleep(0.01)
    count=ser.inWaiting()
    if count>0:
        data=ser.read(count)
        data=binascii.b2a_hex(data).decode('gbk')
        return data_processing_speed(data)
        # print(data)
        # print(data_processing_torque(data))

if __name__ == "__main__":
    from ast import literal_eval
    ser=serial.Serial("COM5",115200)

    position_ref=read_position()
    # torque_control()
    i = 0
    torque_t = "0000"
    try:
        while True:
            # p_t=read_position()
            # torque_control(PD_controller(10,1,30,25,10))
            q_t=read_position()

            if (5<i):
                print("position ", q_t)
                
                print("torque ", int(torque_t.lstrip("0") or "0", 16))
                i=0
            q_dot_t=read_torque()
            torque_t=PD_controller(1,0.5,position_ref,q_t,q_dot_t) # set kd to zero first. because it is dangerous. 

            i+=1
            torque_control(torque_t)
            time.sleep(0.0001)
    except KeyboardInterrupt:
        ser.close()
# print(char_checksum(bytes.fromhex('ECFF')))
