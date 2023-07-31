from timeit import default_timer as timer
from threading import Thread, Event, Lock
from time import sleep
from pymata4.pymata4 import Pymata4
import pygame
import math
import config
from sys import exit



# config the board port
board = Pymata4(config.com)




class RotateSonar(Thread):
    def __init__(self):
        super().__init__()
        self._kill = Event()
        self.lock = Lock()
        
        # maximum angle the servo can reach 
        self.max_angle = config.max_angle
        
        # minimum angle the servo can reach
        self.min_angle = config.min_angle
        
        # total time of the half cycle (from min_angle to max_angle and vice versa) note that the total isnt accurate due to some delay in the response of the motor
        self.total_time = config.time
        
        self.pin = config.servo1_pin
        board.set_pin_mode_servo(self.pin)
        
        # step time is the time between each step
        self.step_time = abs(self.total_time/(self.max_angle-self.min_angle))
        
    def run(self):
        # start the rotation loop
        self.current_angle = 0
        while True:
            # first half cycle from min to max
            for angle in range(self.min_angle, self.max_angle, 1):
                self.write_angle(angle)
                with self.lock:
                    self.current_angle = angle
                # check if there is a kill signal to the thread
                if self._kill.wait(self.step_time):
                    self.return_to_zero(self.step_time)
                    return
            # second half cycle from max to min
            for angle in range(self.max_angle, self.min_angle, -1):
                self.write_angle(angle)
                with self.lock:
                    self.current_angle = angle
                # check if there is a kill signal to the thread
                if self._kill.wait(self.step_time):
                    self.return_to_zero(self.step_time)
                    return
    
    # function to write angles to the servo motor using the pin number
    def write_angle(self, angle):
        board.servo_write(self.pin, angle)
    
    # function to return the servo motor to zero after kill signal
    def return_to_zero(self, step_time):
        for angle in range(self.current_angle, 0, -1):
            self.write_angle(angle)
            self.current_angle = angle
            sleep(self.step_time)  
    
    # get the current angle of the servo motor
    def get_data(self):
        # wait till the lock is unlocked
        while not self.lock.locked():
            with self.lock:
                return self.current_angle         
    
    # set the kill signal
    def kill(self):
        self._kill.set()
        
                
class Sonar(Thread):
    def __init__(self):
        super().__init__()
        self._kill = Event()
        self.lock = Lock()
        
        # set the echo pin 
        self.echo_pin = config.echo_pin
        # set the trigger pin
        self.trigger_pin = config.trigger_pin
        board.set_pin_mode_sonar(self.trigger_pin, self.echo_pin)
        
        # set the offset distance between the center of the motor axis to the center of the sonar sensor
        self.offset_distance = config.sonar_offset_distance
        
        # initialize the Kalman_filter
        self.initialize_data()
        
    # start the measurement loop
    def run(self):
        while True:
            # get current measurement
            self.Yk = self.read_data()[0]
            
            # start prediction step of the kalman filter
            self.prediction_step()
            # start the measurement step of the kalman filter
            self.measurement_step()
            
            with self.lock:
                # get the filterd measurement
                self.measurement = self.correction_step()
            
            # wait for the kill signal if true kill
            if self._kill.wait(0.01):
                return
            
    # get the sensor measurement
    def read_data(self):
        data = board.sonar_read(self.trigger_pin)
        return data
    
    # intialize the kalman filter
    def initialize_data(self):
        # state
        self.Xk = 0
        # feedback
        self.Yk = 0
        # covariance
        self.Pk = 0
        # constants
        self.F = 1
        self.B = 0
        self.H = 1
        # measurement noise covariance
        self.R = config.R
        # process noise covariance
        self.Q = config.Q
        
    # prediction step using kalman filter equations
    def prediction_step(self):
        self.Xk = self.F * self.Xk
        self.Pk = self.Pk + self.Q
        
    # measurement step using kalman filter equations
    def measurement_step(self):
        self.Kk = self.Pk / (self.Pk + self.R)
        
    # correnction step using kalman filter equations
    def correction_step(self):
        self.Xk = self.Xk + self.Kk * (self.Yk - self.Xk)
        self.Pk = (1-self.Kk) * self.Pk
        return self.Xk
    
    
    # get the measuremnts
    def get_data(self):
        # wait until unlocked
        while not self.lock.locked():
            with self.lock:
                return self.measurement + self.offset_distance
    
    # set the kill signal
    def kill(self):
        self._kill.set()
        

# visuals of the radar sweep
class Visuals(Thread):
    def __init__(self, Thread1, Thread2):
        super().__init__()
        self._kill = Event()
        
        self.Thread1 = Thread1
        self.Thread2 = Thread2
        
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)
        
        # set window height
        self.h = config.window_height
        # set window width
        self.w = config.window_width
        # get step time calculated in the Rotate_Sonar Thread
        self.step_time = Thread1.step_time
        # maximum range of the sonar measurements (if a measuremnt distance exceeds the range it will not appear on the screen)
        self.radius = config.range
        # set the angle between the zero position of the motor and the actual zero 0 (in radians)
        self.sonar_offset_angle = config.servo1_offset_angle * math.pi / 180
        # calculate the scale
        self.scale = self.radius/self.h
        
    
    def run(self):
        #intialize pygame module
        pygame.init()
        
        PI = math.pi
        
        # set screen size
        size = [self.w+15, self.h+15]
        screen = pygame.display.set_mode(size)
        
        my_clock = pygame.time.Clock()
        font = pygame.freetype.SysFont("Aerial", 24)
        
        # intialize the angle in radians and the angle in degrees
        angle = 0
        angle_degrees = 0
        # intialize the measuremnt list
        self.measurements = [[0, 0]] * 181
        
        circle = None
        
        # start of the loop
        while not self._kill.is_set():
            # get events
            for event in pygame.event.get():
                # check for quit event
                if event.type == pygame.QUIT:
                    pygame.quit()
                    kill_all()
                    exit()
                    
                # get mouse clicks
                if event.type == pygame.MOUSEBUTTONDOWN:
                    # if left click
                    if event.button == 1:
                        # get the position of the mouse [x, y]
                        pos = pygame.mouse.get_pos()
                        #draw a circle around the position
                        circle = pos
                        # get the true coordinates from pixils to cms by multiplying by the scale
                        x = pos[0] - self.w/2
                        y = pos[1] - self.h
                        x = x * self.scale
                        y = y * self.scale
                        # send to Laser class
                        Laser(x, y).run()
                        
                    # if right click turn off the laser
                    elif event.button == 3:
                        circle = None
                        Laser(0, 0).laser_off()
            
            # draw the outline of the radar
            screen.fill(self.BLACK)
            pygame.draw.circle(screen, self.GREEN, center=[self.w/2, self.h], radius=self.h, width=2, draw_top_left=True, draw_top_right=True)
            pygame.draw.circle(screen, self.GREEN, center=[self.w/2, self.h], radius=self.h/3, width=2, draw_top_left=True, draw_top_right=True)            
            pygame.draw.circle(screen, self.GREEN, center=[self.w/2, self.h], radius=self.h*(2/3), width=2, draw_top_left=True, draw_top_right=True)           
            pygame.draw.line(screen, self.GREEN, start_pos=[0, self.h], end_pos=[self.w, self.h], width=4)
            if circle:
                pygame.draw.circle(screen, self.BLUE, circle, 10)
            # get the x, y coordinates by converting from polar to cartesian (in pixels)
            x = self.h * math.sin(angle) + self.w/2
            y = self.h * math.cos(angle) + self.h

            # draw the sweep line
            pygame.draw.line(screen, self.GREEN, [self.w/2, self.h], [x, y], 2)
            
            # get the distance measured by the sonar
            distance = self.get_measurement() 
            
            # draw the measured values within our range
            self.draw_target(angle_degrees, angle, distance, screen)
            
            # get current sonar angle in degrees
            angle_degrees = self.get_angle()
            # convert from degrees to radians
            angle = ((angle_degrees * PI) / 180) + PI/2
    
            pygame.display.flip()
        pygame.quit()
    
    # function to get the current sonar angle
    def get_angle(self):
        return self.Thread1.get_data()
    
    # function to get the current measurements of the sonar
    def get_measurement(self):
        return self.Thread2.get_data()
    
    # function to draw the measured values within our range on the screen
    def draw_target(self, angle_degrees, angle, distance, screen):
        self.measurements[angle_degrees] = [distance, angle] 
        c=0
        for m in self.measurements:
            # get the relative distance by dividing by the scale (false distance) (in pixils)
            relative_distance = m[0] / self.scale
            # draw the measurements that is within our range
            if relative_distance < self.h:
                # get the cartesian coordinates
                x = relative_distance * math.sin(m[1]) + self.w/2
                y = relative_distance * math.cos(m[1]) + self.h
                try:
                    # get the next measurement in the list
                    m1 = self.measurements[c+1]
                    relative_distance1 = m1[0] / self.scale
                    # get the cartesian coordinates
                    x1 = relative_distance1 * math.sin(m1[1]) + self.w/2
                    y1 = relative_distance1 * math.cos(m1[1]) + self.h
                    # measure the distance between measurement 1 and measurement 2
                    distance = math.sqrt((x-x1)**2 + (y-y1)**2)
                    # if the distance is less than 100 then we can draw a line connecting these two measurements
                    if distance < 50:
                        pygame.draw.line(screen, self.RED, [x,y], [x1,y1], 2)
                except:
                    pygame.draw.line(screen, self.RED, [x,y], [x,y], 7)
            c+=1
    
    # set kill signal
    def kill(self):   
        self._kill.set() 
        
    

# class that control the laser output and rotation
class Laser(Thread):
    def __init__(self, x, y):
        super().__init__()
        # the angle between the zero position of the motor and the actual zero 0 (in radians)
        self.sonar_offset_angle = config.servo1_offset_angle * math.pi / 180
        # set the angle between the zero position of the laser servo motor and the actual zero 0 (in degrees)
        self.laser_offset_angle = config.servo2_offsit_angle
        
        # set the pin numbers
        self.pin_servo = config.servo2_pin
        self.pin_laser = config.laser_pin
        
        # set modes
        board.set_pin_mode_servo(self.pin_servo)
        board.set_pin_mode_digital_output(self.pin_laser)
        
        self.x = x
        self.y = y
        
    def run(self):
        # turn the laser off
        self.laser_off()
        # remove offset angle of the sonar
        self.remove_offsit()
        # get required angle to point the laser to
        angle = self.calculate_angle(self.x,self.y)
        # rotate the laser to the required angle
        self.rotate_laser(angle)
        # turn the laser on
        self.laser_on()
        
    
    # calculate angle using vector rules
    def calculate_angle(self, x, y):
        # set x, y laser
        x_laser = config.relative_x_distance
        y_laser = config.relative_y_distance
        
        # get the vector of the laser to target
        laser_to_target_vector = (x - x_laser, y - y_laser)
        # denominator
        d = laser_to_target_vector[0] * x_laser + laser_to_target_vector[1] * y_laser
        # nominator
        n = math.sqrt(laser_to_target_vector[0]**2 + laser_to_target_vector[1]**2) * math.sqrt(x_laser**2 + y_laser**2)
        # angle using angle between two vectors rule (in radians)
        laser_to_target_angle = math.acos(d / n)
        # convert the angle two degrees
        laser_to_target_angle *= 180 / math.pi
        # substract the offsit angle
        laser_to_target_angle -= self.laser_offset_angle
        return laser_to_target_angle
    
    def remove_offsit(self):
        radius = math.sqrt(self.x**2 + self.y**2)
        angle = math.acos(self.x/radius) - self.sonar_offset_angle
        self.x = radius * math.cos(angle)
        self.y = radius * math.sin(angle)
    
    # function to rotate the laser to disired angle
    def rotate_laser(self, angle):
        board.servo_write(self.pin_servo, int(angle))
    # function to turn on the laser
    def laser_on(self):
        board.digital_write(self.pin_laser, 1)
    # function to turn off the laser
    def laser_off(self):
        board.digital_write(self.pin_laser, 0)
        


Thread1 = RotateSonar()
Thread2 = Sonar()
Thread3 = Visuals(Thread1, Thread2)
 

def main():
    # credits
    print('''author: Mario Morcos
email: mariomorcoswassily@gmail.com''')
    # start threads
    Thread1.start()
    Thread2.start()
    Thread3.start()
    
# send kill signals to all threads
def kill_all():
    Thread1.kill()
    Thread2.kill() 
    Thread3.kill()
    # wait for every thread to terminate
    sleep(6)
    # shut down the board
    board.shutdown()
    
if __name__ == "__main__":
    main()
        


            