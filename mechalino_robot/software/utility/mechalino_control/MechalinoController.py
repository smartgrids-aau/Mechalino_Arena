import msvcrt
from MechalinoControl import MechalinoControl

mechalino_control = MechalinoControl()

print("Press q to quit\nf to move forward\nb to move backward\nl to rotate left\nr to rotate right")

while(True):
    a = msvcrt.getch().decode('utf-8')
    if a == 'q':
        break
    elif a == 'f':
        mechalino_control.forward(3)
    elif a == 'b': 
        mechalino_control.backward(3)
    elif a == 'l':
        mechalino_control.rotate_ccw(90)
    elif a == 'r':
        mechalino_control.rotate_cw(90)

