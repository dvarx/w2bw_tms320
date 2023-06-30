from re import I
import matplotlib.pyplot as plt
from math import sqrt
import serial
import pickle5 as pickle


def read_w2bw_tms320_data(datasource,N=0,is_akm=False):
    """
    this function decodes magnetic field values Bx, By and Bz from either a binary file or a bytes object
    """

    #if a string is passed, open the corresponding file and read the raw bytes
    if type(datasource)==str:
        with open(datasource,"rb") as fptr:
            data=fptr.read()
            read_from_file=True
    else:
        data=datasource
        read_from_file=False

    idx=0
    #find the start of the datastream 
    startidx=0
    while(True):
        #detect start of data frame
        if(data[idx]==0x55 and data[idx+1]==0x55):
            break
        idx+=1
        
    Bxs=[]
    Bys=[]
    Bzs=[]
    Bms=[]
    Bmprev=0
    Bxprev=0
    Byprev=0
    Bzprev=0
    Bxprevprev=0
    Byprevprev=0
    Bzprevprev=0
    correct_prev_x=False
    correct_prev_y=False
    correct_prev_z=False
        
    no_frames_read=0
    errflag=False

    while(not(idx+8>len(data))):
        #find start of the dataframe
        if(not(data[idx]==0x55 and data[idx+1]==0x55)):
            idx+=1
            continue
        #parse Bx
        bx_bytes=bytearray([data[idx+3],data[idx+2]])
        #parse By
        by_bytes=bytearray([data[idx+5],data[idx+4]])
        #parse Bz
        bz_bytes=bytearray([data[idx+7],data[idx+6]])

        if is_akm:
            Bx=tesla_per_bit_akm*int.from_bytes(bx_bytes,"big",signed=True)
            By=tesla_per_bit_akm*int.from_bytes(by_bytes,"big",signed=True)
            Bz=tesla_per_bit_akm*int.from_bytes(bz_bytes,"big",signed=True)
        else:
            Bx=tesla_per_bit*int.from_bytes(bx_bytes,"big",signed=True)
            By=tesla_per_bit*int.from_bytes(by_bytes,"big",signed=True)
            Bz=tesla_per_bit*int.from_bytes(bz_bytes,"big",signed=True)

        Bm=sqrt(Bx**2+By**2+Bz**2)
        dataframe=bytearray([data[idx+n] for n in range(0,8)])

        #detect frame erros
        try:
            #detect error in frame format
            if not(data[idx]==0x55 and data[idx+1]==0x55 and data[idx+8]==0x55 and data[idx+9]==0x55):
                errflag=True
                print("Error:     %s"%(str(dataframe)))
                print("Error-Index: %d , FrameNo: %d"%(idx,no_frames_read))
            else:
                #print(str(dataframe))
                pass
        except Exception:
            pass
        
        Bxs.append(Bx)
        Bys.append(By)
        Bzs.append(Bz)
        Bms.append(Bm)
        #store values
        Bxprev=Bx
        Byprev=By
        Bzprev=Bz
        Bmprev=Bm

        idx+=8
        no_frames_read+=1
        if N>0 and no_frames_read>=N:
            break
    
    if read_from_file:
        fptr.close()
    return {"Bxs":Bxs,"Bys":Bys,"Bzs":Bzs,"Bms":Bms}

def w2bw_read_n_bytes(N,devfptr="/dev/tms320",samplerate=500):
    ser=serial.Serial(devfptr,460800,timeout=N/samplerate*2)
    data_bytes=ser.read(N)
    ser.close()
    return data_bytes

no_frames=50
no_meas_per_frame=100
no_bytes_per_meas=8
fsample=2000

#default resolution for the long range setting
tesla_per_bit=1e-3/7.7
#default resolution for the short range setting
tesla_per_bit=1e-3/30.8

databytes=w2bw_read_n_bytes(no_frames*(no_meas_per_frame)*no_bytes_per_meas,"/dev/ttyUSB0",fsample)

data=read_w2bw_tms320_data(databytes,is_akm=False,N=no_frames*no_meas_per_frame)
#data=read_w2bw_tms320_data_syncframe(databytes)

Bxs=data["Bxs"]
Bys=data["Bys"]
Bzs=data["Bzs"]
Bms=data["Bms"]

# Save data in pickle file
with open('z_10mT_15Hz.txt', 'wb') as f:
    pickle.dump(data,f)    

f.close()

plt.subplot(411)
plt.plot(Bxs)
plt.subplot(412)
plt.plot(Bys)
plt.subplot(413)
plt.plot(Bzs)
plt.subplot(414)
plt.plot(Bms)
plt.show()
