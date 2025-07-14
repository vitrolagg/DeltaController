from .kinematics import InverseKinematics
from .ModbusRTU import mover_motor, set_velocidade
import numpy as np

class DeltaRobot:
    def __init__(self, base_radius, gripper_radius, active_arm, passive_arm):
        self.base_radius = base_radius
        self.gripper_radius = gripper_radius
        self.active_arm = active_arm
        self.passive_arm = passive_arm
        self.theta = [0, 120, -120]
        self.inverse_k = InverseKinematics(
            self.base_radius, self.gripper_radius, self.active_arm, self.passive_arm, theta=self.theta
        )
        self.current_angles = [0, 0, 0]
        self.target_angles = [0, 0, 0]
        self.setpoint_speed = 15

    def calculate_angles(self, xyz):
        if isinstance(xyz, list):
            xyz = np.array(xyz)
        phi = self.inverse_k.inverse(xyz)
        if phi is not None:
            return phi
        
    def speeds(self, current_angles, target_angles, setpoint_speed=None):
        if setpoint_speed is None:
            setpoint_speed = self.setpoint_speed
        
        # Encontra o deslocamento entre o ângulo atual e o ângulo alvo, para os três motores
        deslocamento = [abs(new - current) for new, current in zip(target_angles, current_angles)]

        # Econtra o deslocamento maior entre os três motores
        delta_max = max(deslocamento)

        #Incia a lita de velocidades
        speeds = [0, 0, 0]

        # Calcula o tempo necessário para atingir o ângulo alvo
        if delta_max == 0:
            tempo = 0
        else:
            tempo = delta_max / setpoint_speed
            for i in range(len(deslocamento)):
                if deslocamento[i] > 0:
                    speeds[i] = deslocamento[i] / tempo
                else:
                    speeds[i] = 0

        # Formata cada velocidade para duas casas decimais antes de retornar
        formatted_speeds = [round(speed, 2) for speed in speeds]

        return formatted_speeds
    
    def move_robot(self, x, y, z):

        target_pose = [x, y, z]

        calculated_angles = self.calculate_angles(target_pose)

        if calculated_angles is not None:

            theta1, theta2, theta3 = calculated_angles

            self.target_angles = calculated_angles

            speeds = self.speeds(self.current_angles, self.target_angles, self.setpoint_speed)

            self.current_angles = self.target_angles

            speed1, speed2, speed3 = speeds

            set_velocidade(speed1, 1)
            set_velocidade(speed2, 2)
            set_velocidade(speed3, 3)

            mover_motor(theta1, 1)
            mover_motor(theta2, 2)
            mover_motor(theta3, 3)


            
