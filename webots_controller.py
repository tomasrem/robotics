from controller import Robot 
import math
import numpy as np 
import serial

def run_robot(robot):

    time_step = 64

    # MOTORS
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # GROUND SENSORS
    left_gs = robot.getDevice("gs0")
    left_gs.enable(time_step)
    middle_gs = robot.getDevice("gs1")
    middle_gs.enable(time_step)
    right_gs = robot.getDevice("gs2")
    right_gs.enable(time_step)

    # ENCODERS
    left_encoder = robot.getDevice("left wheel sensor")
    left_encoder.enable(time_step)
    right_encoder = robot.getDevice("right wheel sensor")
    right_encoder.enable(time_step)

    # CONSTANTS    
    ω_wheels_angular_velocity = 6.28 # ω (rpm ((1/(s*60))) 
    wheel_radius = 0.0205 # r (m)
    wheel_cirumference = 2 * wheel_radius * math.pi # (m)
    encoder_num = wheel_cirumference/ω_wheels_angular_velocity # (m * s * 60 )

    # INITIAL VALUES
    state = "IDLE"
    angle = 0
    rotation_counter = 0
    robot_total_displacement_x = 0
    robot_total_displacement_y = 0 
    displacement_after_turn_x = 0
    displacement_after_turn_y = 0
    stop_time = robot.getTime()
    node_counter = 0
    reset_left_encoder_value = 0.0
    reset_right_encoder_value = 0.0


    # UART_COMMUNICATION
    try:
        ser =  serial.Serial(port='COM5', baudrate=115200, timeout=5) 
    except:
        print("Communication failed. Check the cable connections and serial settings 'port' and 'baudrate'.")
        raise

    while robot.step(time_step) != -1:
        match state:
            case "IDLE":
                # SET LINEAR VELOCITY TO 0.0
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
 
                # SEND STATE TO THE ESP-32
                message = ""
                message = "IDLE\n"
                ser.write(message.encode('utf-8'))

                # READING THE PATH AS A STRING
                path_string = str(ser.readline(), 'UTF-8')[:-1]

                # MAKING THE PATH AN ARRAY/LIST AGAIN
                path_list = list(path_string)
                print(path_list)

                # HOW MANY NODES THE ROBOT WILL GO THROUGH
                nodes_to_travel = len(path_list)
                print(nodes_to_travel)

                # COORDINATES OF EACH NODE (EACH COORDINATE HAS A LETTER ASSOCIATED)
                A_coordinate = [0, 0]
                B_coordinate = [0, 0.1]
                C_coordinate = [0, 0.2]
                D_coordinate = [0, 0.3]
                E_coordinate = [0, 0.5]
                F_coordinate = [0, 1]
                G_coordinate = [0.25, 0]
                H_coordinate = [0.1667, 0.5]
                I_coordinate = [0.1667, 1]
                J_coordinate = [0.25, 0.5]
                K_coordinate = [0.25, 1]
                L_coordinate = [0.33, 0]
                M_coordinate = [0.33, 0.5]
                N_coordinate = [0.5, 0]
                Ñ_coordinate = [0.5, 0.5]
                O_coordinate = [0.5, 0.7]
                P_coordinate = [0.5, 0.8]
                Q_coordinate = [0.5, 0.9]
                R_coordinate = [0.5, 1]

                path_in_coordinates = []

                # RELATING THE COORDINATES IN NUMBERS TO THE NODES/LETTERS
                for i in range(nodes_to_travel):
                    if path_list[i] == "A":
                        path_in_coordinates.append(A_coordinate)
                    elif path_list[i] == "B":
                        path_in_coordinates.append(B_coordinate)
                    elif path_list[i] == "C":
                        path_in_coordinates.append(C_coordinate)
                    elif path_list[i] == "D":
                        path_in_coordinates.append(D_coordinate)
                    elif path_list[i] == "E":
                        path_in_coordinates.append(E_coordinate)
                    elif path_list[i] == "F":
                        path_in_coordinates.append(F_coordinate)
                    elif path_list[i] == "G":
                        path_in_coordinates.append(G_coordinate)
                    elif path_list[i] == "H":
                        path_in_coordinates.append(H_coordinate)
                    elif path_list[i] == "I":
                        path_in_coordinates.append(I_coordinate)
                    elif path_list[i] == "J":
                        path_in_coordinates.append(J_coordinate)
                    elif path_list[i] == "K":
                        path_in_coordinates.append(K_coordinate)
                    elif path_list[i] == "L":
                        path_in_coordinates.append(L_coordinate)
                    elif path_list[i] == "M":
                        path_in_coordinates.append(M_coordinate)
                    elif path_list[i] == "N":
                        path_in_coordinates.append(N_coordinate)
                    elif path_list[i] == "Ñ":
                        path_in_coordinates.append(Ñ_coordinate)
                    elif path_list[i] == "O":
                        path_in_coordinates.append(O_coordinate)
                    elif path_list[i] == "P":
                        path_in_coordinates.append(P_coordinate)
                    elif path_list[i] == "Q":
                        path_in_coordinates.append(Q_coordinate)
                    elif path_list[i] == "R":
                        path_in_coordinates.append(R_coordinate)

                print(path_in_coordinates)

                print(node_counter)

                # IF THE ROBOT VISITED ALL THE NODES, THEN BREAK
                if node_counter + 1 == nodes_to_travel:
                    break
                
                # LOGIC TO KNOW WHERE THE ROBOT NEEDS TO GO NEXT (LEFT, RIGHT OR FORWARD)
                if path_in_coordinates[node_counter][0] == path_in_coordinates[node_counter + 1][0] and path_in_coordinates[node_counter][1] < path_in_coordinates[node_counter + 1][1] and (rotation_counter == 0 or  rotation_counter == 2):
                    print("left,")
                    print("AGAIN")
                    stop_time = robot.getTime()
                    displacement_after_turn_x = robot_total_displacement_x
                    displacement_after_turn_y = robot_total_displacement_y
                    state = "left"
                    
                elif path_in_coordinates[node_counter][0] < path_in_coordinates[node_counter + 1][0] and path_in_coordinates[node_counter][1] == path_in_coordinates[node_counter + 1][1] and (rotation_counter == 1 or  rotation_counter == 3):
                    print("right")
                    ("AGAIN")
                    stop_time = robot.getTime()
                    displacement_after_turn_x = robot_total_displacement_x
                    displacement_after_turn_y = robot_total_displacement_y
                    state = "right"
                    
                else:
                    state = "forward"
                    print("forward,")
                    print("AGAIN")             


            case "forward":
                
                # MESSAGE FOR ESP-32
                message = ""
                message = "forward\n"
                print(message)
                ser.write(message.encode('utf-8'))
                

                # ACCURATE ROTATION ANGLE OF THE ROBOT
                if rotation_counter == 0:
                    angle = 0
                elif rotation_counter == 1:
                    angle = (math.pi)/2
                elif rotation_counter == 2:
                    angle = math.pi
                elif rotation_counter == 3:
                    angle = (3*(math.pi))/2
                elif rotation_counter == 4:
                    angle = 2 * (math.pi)

                # GET GROUND SENSOR VALUES
                left_gs_value = left_gs.getValue()
                right_gs_value = right_gs.getValue()
                middle_gs_value = middle_gs.getValue()
                
                # CHECKS IF MIDDLE GROUND SENSOR DETECTS BLACK LINE
                if middle_gs_value < 600:

                    #SET LINEAR VELOCITY TO ROBOT
                    left_motor.setVelocity(ω_wheels_angular_velocity)
                    right_motor.setVelocity(ω_wheels_angular_velocity)

                    # ENCODERS, SENSORS THAT DETECT THE ANGULAR VELOCITY OF WHEELS
                    left_encoder_value = left_encoder.getValue() - reset_left_encoder_value
                    right_encoder_value = right_encoder.getValue() - reset_right_encoder_value

                    # CALCULATING METERS
                    robot_left_displacement = left_encoder_value * encoder_num # m * s *60 * (1/s) * (1/60) = m
                    robot_right_displacement = right_encoder_value * encoder_num

                    # DISPLACEMENT OF BOTH WHEELS, ANGLE GIVES THE COORDINATE FOR EACH
                    robot_total_displacement_x = ((robot_left_displacement + robot_right_displacement)/2) * math.cos(angle) + displacement_after_turn_x #+ old_tot_dis_x falta variable y hacerla para x e y
                    robot_total_displacement_y = ((robot_left_displacement + robot_right_displacement)/2) * math.sin(angle) + displacement_after_turn_y 

                    print("--------------------------")
                    print(state)

                    print ("x:", robot_total_displacement_x, "meters", "y:", robot_total_displacement_y, "meters" , "angle:", angle, "rad")
                    print ("left gs value:", left_gs_value, "middle gs value", middle_gs_value, "right gs value", right_gs_value)  

                # LINE DETECTION FOR GOING RIGHT
                elif (left_gs_value > 600 and middle_gs_value > 600 and right_gs_value < 600)  :
                    left_motor.setVelocity((ω_wheels_angular_velocity)/2)
                    right_motor.setVelocity((-ω_wheels_angular_velocity)/2)

                # LINE DETECTION FOR GOING LEFT 
                elif(right_gs_value > 600  and middle_gs_value > 600 and left_gs_value < 600):
                    left_motor.setVelocity((-ω_wheels_angular_velocity)/2.5)
                    right_motor.setVelocity((ω_wheels_angular_velocity)/2.5)

                # COORDINATE ARRIVAL
                if (robot_total_displacement_x ) >= path_in_coordinates[node_counter + 1][0] and rotation_counter == 0:
                    node_counter = node_counter + 1 
                    state = "IDLE"

                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)

                elif (robot_total_displacement_y) >= path_in_coordinates[node_counter + 1][1] and rotation_counter == 1:
                    node_counter = node_counter + 1 
                    state = "IDLE"

                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                
                # DETECTION OF NO BLACK LINE
                elif (middle_gs_value > 600 and left_gs_value > 600 and right_gs_value > 600) and (robot_total_displacement_y + 0.05) >= path_in_coordinates[node_counter + 1][1] and rotation_counter == 1:
                    node_counter = node_counter + 1 
                    state = "IDLE"

                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)

                elif (middle_gs_value > 600 and left_gs_value > 600 and right_gs_value > 600) and (robot_total_displacement_x + 0.05) >= path_in_coordinates[node_counter + 1][0] and rotation_counter == 0:
                    node_counter = node_counter + 1 
                    state = "IDLE"

                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)

            case "left":

                
                message = ""

                left_gs_value = left_gs.getValue()
                right_gs_value = right_gs.getValue()
                middle_gs_value = middle_gs.getValue()

                #TIME FOR ROTATION
                time = robot.getTime()
                
                # ROTATION VELOCITY
                left_motor.setVelocity(-ω_wheels_angular_velocity)
                right_motor.setVelocity(ω_wheels_angular_velocity)

                left_encoder_value = left_encoder.getValue()
                right_encoder_value = right_encoder.getValue()

                left_radians_div_seconds = (left_encoder_value * 2 * math.pi)/ 60
                right_radians_div_seconds = (right_encoder_value * 2 * math.pi)/ 60 

                robot_left_displacement = left_encoder_value * encoder_num # m * s *60 * (1/s) * (1/60) = m
                robot_right_displacement = right_encoder_value * encoder_num
            
                # DISPLACEMENT OF BOTH WHEELS, ANGLE GIVES THE COORDINATE FOR EACH
                robot_total_displacement_x = ((robot_left_displacement + robot_right_displacement)/2) * math.cos(angle) #+ old_tot_dis_x falta variable y hacerla para x e y
                robot_total_displacement_y = ((robot_left_displacement + robot_right_displacement)/2) * math.sin(angle) 

                print("--------------------------")
                print(state)

                print ("x:", robot_total_displacement_x, "meters", "y:", robot_total_displacement_y, "meters" , "angle:", angle, "rad")
                print ("left gs value:", left_gs_value, "middle gs value", middle_gs_value, "right gs value", right_gs_value)   

                message = "left\n"
                print(message)
                ser.write(message.encode('utf-8'))

                if right_radians_div_seconds > 0 and  (time - stop_time) >= (math.pi/1.5)/(right_radians_div_seconds):
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    rotation_counter = rotation_counter + 1
                    reset_left_encoder_value = left_encoder.getValue()
                    reset_right_encoder_value = right_encoder.getValue()
                    state = "forward"

            case "right":
                
                message = ""

                message = "right\n"
                print(message)
                ser.write(message.encode('utf-8')) 

                left_gs_value = left_gs.getValue()
                right_gs_value = right_gs.getValue()
                middle_gs_value = middle_gs.getValue()

                time = robot.getTime()
                
                left_motor.setVelocity(ω_wheels_angular_velocity)
                right_motor.setVelocity(-ω_wheels_angular_velocity)

                left_encoder_value = left_encoder.getValue()
                right_encoder_value = right_encoder.getValue()

                left_radians_div_seconds = (left_encoder_value * 2 * math.pi)/ 60
                right_radians_div_seconds = (right_encoder_value * 2 * math.pi)/ 60 

                robot_left_displacement = left_encoder_value * encoder_num # m * s *60 * (1/s) * (1/60) = m
                robot_right_displacement = right_encoder_value * encoder_num
            
                # DISPLACEMENT OF BOTH WHEELS, ANGLE GIVES THE COORDINATE FOR EACH
                robot_total_displacement_x = ((robot_left_displacement + robot_right_displacement)/2) * math.cos(angle) - robot_total_displacement_y  + displacement_after_turn_x #+ old_tot_dis_x falta variable y hacerla para x e y
                robot_total_displacement_y = ((robot_left_displacement + robot_right_displacement)/2) * math.sin(angle) - robot_total_displacement_x  + displacement_after_turn_y #

                print("--------------------------")
                print(state)

                print ("x:", robot_total_displacement_x, "meters", "y:", robot_total_displacement_y, "meters" , "angle:", angle, "rad")
                print ("left gs value:", left_gs_value, "middle gs value", middle_gs_value, "right gs value", right_gs_value)  


                if right_radians_div_seconds > 0 and  (time - stop_time) >= (math.pi/0.8)/(right_radians_div_seconds):
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    rotation_counter = rotation_counter - 1
                    reset_left_encoder_value = left_encoder.getValue()
                    reset_right_encoder_value = right_encoder.getValue()
                    state = "forward"          

if _name_ == "_main_":
    my_robot = Robot()
    run_robot(my_robot)