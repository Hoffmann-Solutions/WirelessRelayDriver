import tkinter
import serial
ser = serial.Serial()
ser.baudrate = '9600'
ser.port = 'COM13'
ser.open()
print (ser.is_open)
    
top=tkinter.Tk()

def send_program():
    mode = var.get()
    if mode == "RX":
        ser.write(b'1')
        Out = ser.readline()
        newOut = Out.decode("UTF-8")
        print(newOut)
    elif mode == "TX":
        ser.write(b'2')
        Out = ser.readline()
        newOut = Out.decode("UTF-8")
        print(newOut)
    
var = tkinter.StringVar(top)
var.set("RX")

L1 = tkinter.Label(top, text = "Mode")
L1.grid(row = 0, column=0)
option = tkinter.OptionMenu(top,var,"RX","TX")
option.grid(row = 0, column = 1)
B1 = tkinter.Button(top, bd = 5, text = "Program", command = send_program)
B1.grid(row = 1, column = 0, columnspan = 2, sticky = 'n,e,s,w')



top.mainloop()
