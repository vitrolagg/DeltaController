from pymodbus.client import ModbusSerialClient

# Configurações da rede modbus
client = ModbusSerialClient(
        port='/dev/ttyUSB0',
        baudrate=19200,
        timeout=1,
        parity='N',
        stopbits=1,
        bytesize=8
    )

# Tenta conectar ao dispositivo Modbus RTU
if client.connect():
    print("Conectado ao dispositivo Modbus RTU")

else:
    print("Erro ao conectar ao dispositivo Modbus RTU")

# Função para fechar a conexão
def client_close():
    if client:
        client.close()
        print("Conexão fechada")

"""Parâmetros do servo motor"""

#Número de passos por rotação
passos_por_rotacao=10000

"""Funções de escrita do Modbus RTU"""

#Função de escrita em um registrador modbusRTU
def modbus_write(register, data, slaveID):
    client.write_register(register, data, slave= slaveID)


# Função para escrever um valor signed de 32 bits em dois registradores Modbus consecutivos em ordem little-endian (baixo, alto)
def modbus_write_32(register, data, slaveID):

    # Garante que o valor é um inteiro de 32 bits
    data = int(data) & 0xFFFFFFFF
    low = data & 0xFFFF
    high = (data >> 16) & 0xFFFF
    client.write_registers(register, [low, high], slave=slaveID)


"""Funções simples"""

#Função para habilitar o servo selecionado
def enable(servo):
    modbus_write(register= 1008, data= 1, slaveID= servo)

#Função para desabilitar o servo selecionado
def disable(servo):
    modbus_write(register= 1008, data= 0, slaveID= servo)

#Função de velocidade positva 
def velocidade_positiva(vel, servo):
    modbus_write(register= 1880, data= vel, slaveID= servo)

#Função de velocidade negativa
def velocidade_negativa(vel, servo):

    if vel < 0:
        vel = (1 << 16) + vel  # Converte para complemento de dois

    modbus_write(register= 1882, data= vel, slaveID= servo)

#Função para set de homing de um único servo
def set_homing(servo):
    modbus_write(register= 2030, data= 1, slaveID= servo)

#Função para converter passos para ângulos
def passos_para_angulo(angulo):    

    passo =  (angulo * passos_por_rotacao) / 360

    passo = int(passo)

    # print(passo)

    return passo


"""Funções compostas"""

#velocidade para os dois sentidos de giro
def set_velocidade(vel, servo):

    vel = int(vel)
    
    velocidade_positiva(vel, servo)

    vel = -vel

    velocidade_negativa(vel, servo)

#Função para mover o motor para um ângulo específico
def mover_motor(angulo, servo):
    passo = passos_para_angulo(angulo)
    # print(f"Movendo o servo {servo} para {passo} passos (32 bts)")
    modbus_write_32(register=1824, data=passo, slaveID=servo)