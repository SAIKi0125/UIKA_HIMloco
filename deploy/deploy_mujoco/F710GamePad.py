import pygame
from pygame.locals import *
from time import sleep

class F710GamePad:
    
    def __init__(self):

        try:
            pygame.joystick.init()
            self.joystick0 = pygame.joystick.Joystick(0)
            self.joystick0.init()
            pygame.init()
            print("Gamepad Detected")

        except pygame.error:
            print("Gamepad Not Found")
            return

        # default parameters
        self.x0, self.y0 = 0.0, 0.0 # left joystick
        self.x1, self.y1 = 0.0, 0.0 # right joystick
        self.hat = [0,0]            # hat
        self.button = []            # buttons
        self.triggerL = 0.0         # left trigger
        self.triggerR = 0.0         # right trigger

        # default setting
        self.joyL_max = 100     # left joystick at default has output of +/- 100 
        self.joyR_max = 100
        self.trigL_max = 100
        self.trigR_max = 100

        self.release_after = 2  # time to automatically release pressed buttons
        self.freq = 20          # sampling frequency
        self.round_output = 0
        self.count = 0
        self.os = 'windows' # 

    # obtain all input values from the gamepad
    def GetValues(self):
        pygame.init()
        self.count += 1
        # numbering is different in Windows10 and Ubuntu
        if self.os.lower() == 'windows': axis0, axis1, axis2, axis3, axis4, axis5 = 0, 1, 2, 3, 4, 5
        else: axis0, axis1, axis2, axis3, axis4, axis5 = 0, 1, 3, 4, 2, 5

        # sleep(1/self.freq)
        eventlist = pygame.event.get()

        for event in eventlist:
            if event.type == pygame.locals.JOYAXISMOTION and (event.axis == axis0 or event.axis == axis1):
                self.x0, self.y0 = round(self.joystick0.get_axis(axis0)*self.joyL_max,self.round_output), -1*round(self.joystick0.get_axis(axis1)*self.joyL_max,self.round_output)

            if event.type == pygame.locals.JOYAXISMOTION and (event.axis == axis2 or event.axis == axis3):
                self.x1, self.y1 = round(self.joystick0.get_axis(axis2)*self.joyR_max,self.round_output), -1*round(self.joystick0.get_axis(axis3)*self.joyR_max,self.round_output)

            if event.type == pygame.locals.JOYAXISMOTION and (event.axis == axis4):
                self.triggerL = round((self.joystick0.get_axis(axis4)+1)*self.trigL_max/2,self.round_output) 

            if event.type == pygame.locals.JOYAXISMOTION and (event.axis == axis5):
                self.triggerR = round((self.joystick0.get_axis(axis5)+1)*self.trigR_max/2,self.round_output) 

            if event.type == pygame.locals.JOYHATMOTION:
                self.hat = self.joystick0.get_hat(0)

            if event.type == pygame.locals.JOYBUTTONDOWN:
                if event.button not in self.button: self.button.append(event.button)

            if event.type == pygame.locals.JOYBUTTONUP: 
                try: self.button.remove(event.button)
                except: pass

        output = self.x0,self.y0,self.x1,self.y1,self.triggerL,self.triggerR,self.hat,self.button

        pygame.event.clear()
        if self.count > self.freq*self.release_after: 
            self.button = [] # release buttons after a while
            self.count = 0
        return output


    # - get values from the specified input method/s such as left/right joystick, triggers, and buttons
    # - to obtain inputs, simply set desired input parameters to 1 (i.e. joyL=1 to obtain values from left joystick)
    # - the sampling frequency can be changed with freq
    # output format: [[left joystick],[righ joystick],left trigger, right trigger, [hat], [buttons]]
    def GetInput(self,joyL=0,joyR=0,trigL=0,trigR=0,hat=0,buttons=0,
              joyL_max=100,joyR_max=100,trigL_max=100,trigR_max=100,
              freq=20,os='windows',release_after=2,round_output=0):

        # default range(+/-) of joystick reading
        self.joyL_max = joyL_max
        self.joyR_max = joyR_max
        self.trigL_max = trigL_max
        self.trigR_max = trigR_max

        self.freq = freq    # sampling frequency
        self.release_after = release_after  # automatic release time of buttons in seconds
        self.round_output = round_output    # rounding of the output
        
        output = []
        inputs = self.GetValues() 

        if joyL: output.append(list(inputs[0:2]))
        if joyR: output.append(list(inputs[2:4]))
        if trigL: output.append(inputs[4])
        if trigR: output.append(inputs[5])
        if hat: output.append(list(inputs[6]))
        if buttons: output.append(inputs[7])
        else: pass
        return output



# example of usage
if __name__ == '__main__':

    gamepad = F710GamePad()
    while True:
        #[[0.0, 0.0], [0.0, 0.0], 0.0, 0.0, [0, 0], []]
        sleep(0.1)
        # values = gamepad.GetInput(joyL=1,joyR=1,buttons=1,round_output=3)
        values = gamepad.GetInput(joyL=1,joyR=1,trigL=1,trigR=1,buttons=1,hat=1,joyL_max=100,os='linux')
        # print("x0 = %4.3f y0 = %4.3f x1 = %4.3f y1 = %4.3f " % (values[0][0], values[0][1], values[1][0],values[1][1]))
        print(values)