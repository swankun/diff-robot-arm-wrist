import serial
import time
import sys

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None)
# ser.open()
time.sleep(1.0)
print("Connected!")

# for i in range(0, 1000):
#     # msg = b'\x3e\x9b\x03\x00\xdc'
#     # msg = b'\x3e\x9a\x03\x00\xdb'
#     msg = b'\x3e\x12\x03\x00\x53'
#     outgoing = ser.write(msg)

# msg = b'\x3e\x9b\x03\x00'
# chksum = sum(msg)
# msg += chksum.to_bytes(1, 'big')
# print(msg)

id = 1
while True:
    try:
        incoming = ser.read(1)
        print(time.time_ns(), '(in) ', incoming.hex())
        if incoming.hex() == '00':
            time.sleep(1e-6)
            msg = b'\x3e\x12' + id.to_bytes(1, 'big') + b'\x00'
            chksum = sum(msg)
            msg += chksum.to_bytes(1, 'big')
            outgoing = ser.write(msg)
            
            readable = " ".join(["{:02X}".format(x) for x in msg])
            # print("Sent", outgoing, "bytes:", readable, "\n")
            print(time.time_ns(), '(out)', readable)
            if id < 20:
                id = id + 1
            else:
                id = 1
        # time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nBye")
        ser.close()
        sys.exit()
    