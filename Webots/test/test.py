from controller import Robot

#inicjalizacja robota
robot = Robot()

#pobranie uchwytu na koła
left_wheel = robot.getDevice('left wheel motor')
right_wheel = robot.getDevice('right wheel motor')

#ustawienie maksymalnej prędkości kół
max_speed = 6.28
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))
left_wheel.setVelocity(0)
right_wheel.setVelocity(0)


def move_forward():
    left_wheel.setVelocity(max_speed)
    right_wheel.setVelocity(max_speed)

def move_backward():
    left_wheel.setVelocity(-max_speed)
    right_wheel.setVelocity(-max_speed)

def turn_left():
    left_wheel.setVelocity(-max_speed)
    right_wheel.setVelocity(max_speed)

def turn_right():
    left_wheel.setVelocity(max_speed)
    right_wheel.setVelocity(-max_speed)

# Główna pętla programu
while robot.step(64) != -1:
    move_forward()
    robot.step(1000) 
    
    move_backward()
    robot.step(1000)  
    
    turn_left()
    robot.step(1000)  
    
    turn_right()
    robot.step(1000)  
    
    #stop
    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)
    