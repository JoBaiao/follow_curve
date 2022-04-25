#!/usr/bin/env python3

import numpy as np
from scipy.linalg import norm
from random import random, randint
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

class TurtleBot:
    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    ###################################################################################################
    #Callback da Pose toda vez que é publicada.
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)


    ###################################################################################################
    # Encontra ponto mais próximo dentro do trajeto
    def pontomaisprox(self,dxatual,dyatual,C):
        d = [] # Distâncias
        
        for i in list(range(0,len(C[1]))):
            d.append(norm([self.pose.x + dxatual - C[0][i] , self.pose.y + dyatual - C[1][i]]))
        
        dmin = min(d) # Distância mínima
        j = d.index(min(d)) # Indíce correspondente à distância mínima
        
        p_star = [C[0][j],C[1][j]] # Ponto mais próximo
        
        norma = norm(np.array([p_star[0] - (self.pose.x + dxatual) ,p_star[1] - (self.pose.y + dyatual)])) # Norma da distância p p*
        
        N = [(p_star[0] - (self.pose.x + dxatual))/norma,(p_star[1] - (self.pose.y + dyatual))/norma] # Vetor normal
        
        if j == (len(C[1])-1):
            T = [C[0][0] - C[0][j] , C[1][0] - C[1][j]] 
        else:
            T = [C[0][j+1] - C[0][j] , C[1][j+1] - C[1][j]] # Caso especial

        T = np.array(T)/norm(T) #Vetor tangencial
        
        return [N,T,dmin]


    ###################################################################################################
    # Composição Vetor Normal e Tangente
    def composicao_de_vetores (self,N,T,beta,dmin):
        
        G = (2/np.pi)*np.arctan(beta*dmin)
        H = np.sqrt(1-G**2)
        
        return 0.7*(G*np.array(N)+H*np.array(T))


    ###################################################################################################
    # Lei de Controle
    def control(self,C):
        
        pmed = np.array([self.pose.x + 0.1 - 0.2*random(), self.pose.y + 0.1 - 0.2*random()])

        deslocamento_atual = np.array([self.v*np.cos(self.pose.theta)*self.dt, self.v*np.sin(self.pose.theta)*self.dt])
        
        [N,T,dmin] = self.pontomaisprox(deslocamento_atual[0],deslocamento_atual[1],C)
        velocidade_desejada = self.composicao_de_vetores (N,T,self.beta,dmin)
        
        [N,T,dmin] = self.pontomaisprox(deslocamento_atual[0],deslocamento_atual[1],C)
        velocidade_desejada_futura = self.composicao_de_vetores (N,T,self.beta,dmin)
       
        derivada_velocidade = (velocidade_desejada_futura - velocidade_desejada)/self.dt
       
        w_max = 1
        k = 2 

        # Matriz
        M = np.array([[self.v*np.cos(self.pose.theta),self.v*np.sin(self.pose.theta)],
                      [-np.sin(self.pose.theta),np.cos(self.pose.theta)]])

        controlador_prop = -k*np.array([self.v*np.cos(self.pose.theta)-velocidade_desejada[0],self.v*np.sin(self.pose.theta)-velocidade_desejada[1]])

        acc_desired = [derivada_velocidade[0]+controlador_prop[0],derivada_velocidade[1]+controlador_prop[1]]



        [a,w] = np.matmul(M/self.v, acc_desired)
        
        if w > w_max:
            w = w_max
        elif w < -w_max:
            w = -w_max
        
        v = self.v + a*self.dt
        


        print(f"velocidade linear: {v}\nvelocidade angular: {w}\n")

        return [v,w]


    ###################################################################################################
    # Move Tartaruga Função Principal
    def move_turtle(self):
        vel = Twist()

        # Cria curva
        theta = np.arange(0, 2*np.pi, 0.01)

        C = [[],[]]

        for theta1 in theta:
            C[0].append(2*np.cos(theta1)+0.2 * 2*np.sin(2*theta1)+0.05*np.sin(4*theta1) + 5.5)
            C[1].append(2*np.sin(theta1)-0.2 * 2*np.cos(2*theta1)-0.05*np.sin(4*theta1) + 5.5)

        # Velocidades Iniciais
        self.v = 1
        self.w = 0.1

        # Parâmetros
        self.dt = 0.03
        self.beta = 3

        while not rospy.is_shutdown():
            [self.v,self.w] = self.control(C)

            # Linear velocity in the x-axis.
            vel.linear.x = self.v
            vel.linear.y = 0
            vel.linear.z = 0

            # Angular velocity in the z-axis.
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = self.w

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel)

            # Publish at the desired rate.
            self.rate.sleep()




###################################################################################################
# Main Function
if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move_turtle()
        
    except rospy.ROSInterruptException:
        pass
