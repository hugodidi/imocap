import numpy as np
from scipy.signal import butter, lfilter
from collections import deque
import math
import asyncio
import threading
import time
import queue
from scipy.signal import butter, filtfilt, savgol_filter
import pika 
import json
import time
import multiprocessing
import matplotlib.pyplot as plt

class IMU:
    def __init__(self, key_binding):
        super().__init__()
        self.conn = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
        self.channel = self.conn.channel() 
        self.channel.exchange_declare(exchange='topic_Proc', exchange_type='topic')
        self.channel.exchange_declare(exchange='topic_BT', exchange_type='topic')
        self.result = self.channel.queue_declare('', exclusive=True)
        self.key_binding = key_binding
        self.queue_name = self.result.method.queue 
        self.channel.queue_bind(exchange='topic_BT', queue=self.queue_name, routing_key=self.key_binding)
        self.process = None
        self.stop_event = multiprocessing.Event()                              
        self.aXread = []
        self.aYread = []
        self.aZread = []
        self.gXread = []
        self.gYread = []
        self.gZread = []
        self.aX0 = []
        self.aY0 = []
        self.aZ0 = []
        self.gX0 = []
        self.gY0 = []
        self.gZ0 = []
        self.mX0 = []
        self.mY0 = []
        self.mZ0 = []
        self.dt_save = []
        self.num = 0.0
        self.time_cicle = []
        self.time_block = 0.0
        self.Temp = []
        self.cont_w=0
        self.cont = None
        self.time_zero = 0
        self.conteo = 0
        self.inicio=False
        self.inicio_int = 0
        self.final_int=0
        self.cronometro_int = 0
        self.a0 = [0.0,0.0,0.0,0.0]
        self.g0 = [0.0,0.0,0.0]
        self.m0 = [0.0,0.0,0.0]
        self.aglobalRaw = [[],[],[]]
        self.gglobalRaw = [[],[],[]]
        self.aglobal = [[],[],[]]
        self.gglobal = [[],[],[]]
        self.quaternion = [[],[],[],[]]
        self.Euler = [[],[],[]]
        self.AccGlobal = [[],[],[]]
        self.velGlobal = [[],[],[]]
        self.VelPlot = [[],[],[]]
        self.velGlobal_filt = [[],[],[]]
        self.Eulerplot=[]
        self.accGlobalplot = []
        self.Datawrite = [[],[],[],[],[],[],[]]
        self.Datawrite_sync = [[],[],[],[]]
        self.Datawrite_q = asyncio.Queue()
        self.flag_buffer = False
        self.theta0 = 0.0
        Q = np.array([[0.0001]])             # Ruido del proceso
        R = np.array([[0.025]])              # Ruido de la medición
        P = np.array([[1]])                # Covarianza inicial
        v0 = 0                             # Velocidad inicial
        umbral = 1.0                      # Umbral de aceleración para fusionar con 0
        self.running = True
        self.calibration = False
        self.num_cal = 250
        self.mag_cal = True
        ###

        self.Qt_w= []
        self.Qt_x= []
        self.Qt_y= []
        self.Qt_z= []

        ###
        
        self.plotTime = []
        self.Pitch = []
        self.Roll = []
        self.Yaw = []


    def MadgwickMagPrediction(self,qt_1, dq, beta, Ebt, Sm):
        qw, qx, qy, qz= qt_1
        bw, bx, by, bz = Ebt
        mx, my, mz = Sm
        #fb
        f1=2*bx*(1/2-qy**2-qz**2)+2*bz*(qx*qz-qw*qy)-mx
        f2=2*bx*(qx*qy-qw*qz)+2*bz*(qw*qx+qy*qz)-my
        f3=2*bx*(qw*qy+qx*qz)+2*bz*(1/2-qx**2-qy**2)-mz
        fb = np.array([[f1],[f2],[f3]])
        #Jb
        Jb = np.array([
            [-2*bz*qy, 2*bz*qz, -4*bx*qy-2*bz*qw, -4*bx*qz+2*bz*qx],
            [-2*bx*qz+2*bz*qx, 2*bx*qy+2*bz*qw, 2*bx*qx+2*bz*qz, -2*bx*qw+2*bz*qy],
            [2*bx*qy, 2*bx*qz-4*bz*qx, 2*bx*qw-4*bz*qy, 2*bx*qx]
            ])
        Jb = Jb.T
        
        gradF = np.dot(Jb,fb)/np.linalg.norm(np.dot(Jb,fb))
        dQmag = np.dot(beta,gradF)
        dQmag = np.array([dQmag[0][0],dQmag[1][0],dQmag[2][0],dQmag[3][0]])
        Qmag = qt_1 + (dq - dQmag)*self.dt
        return Qmag
    
    def plotQuat(self,w,x,y,z,name:str):
        sample_interval = 0.01
        n = len(x)
        t = np.arange(n) * sample_interval
        plt.figure()
        plt.plot(t, w,  label=f'q[w]')
        plt.plot(t, x,  label=f'q[x]')
        plt.plot(t, y,  label=f'q[y]')
        plt.plot(t, z,  label=f'q[z]')
        # Etiquetas y título
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Value")
        plt.title(name)

        # Leyenda y rejilla
        plt.legend()
        plt.grid(True)

    def plotVector(self,x,y,z,name:str):
        sample_interval = 0.01
        n = len(x)
        t = np.arange(n) * sample_interval
        plt.figure()
        plt.plot(t, x,  label=f'v[x]')
        plt.plot(t, y,  label=f'v[y]')
        plt.plot(t, z,  label=f'v[z]')
        # Etiquetas y título
        plt.xlabel("Tiempo (s)")
        plt.ylabel("º")
        plt.title(name)

        # Leyenda y rejilla
        plt.legend()
        plt.grid(True)

    def readData(self,data_to_process):
        #print(str(data_to_process))
        if len(data_to_process) == 9:
            self.mXread=float(data_to_process[0])*0.15
            self.mYread=float(data_to_process[1])*0.15
            self.mZread=float(data_to_process[2])*0.15
            self.aXread=float(data_to_process[3])*9.80665/16384
            self.aYread=float(data_to_process[4])*9.80665/16384
            self.aZread=float(data_to_process[5])*9.80665/16384
            self.gXread=float(data_to_process[6])*math.pi/(131*180)
            self.gYread=float(data_to_process[7])*math.pi/(131*180)
            self.gZread=float(data_to_process[8])*math.pi/(131*180)
            # print(round(self.aXread,2),"\t",
                #   round(self.aYread,2),"\t",
                #   round(self.aZread,2),"\t",
                #   round(self.gXread,2),"\t",
                #   round(self.gYread,2),"\t",
                #   round(self.gZread,2))
            # self.time_cicle.append(data_to_process[11])
            self.time_cicle.append(self.conteo*10+10) #provisional
        #print(str(self.time_cicle))
        if len(self.time_cicle) == 2:
            self.dt = (self.time_cicle[1]-self.time_cicle[0])/1000
            self.time_cicle.pop(0)

    def initialCalibration(self, datos):
        self.readData(datos)
        self.aX0.append(self.aXread)
        self.aY0.append(self.aYread)
        self.aZ0.append(self.aZread)
        self.gX0.append(self.gXread)
        self.gY0.append(self.gYread)
        self.gZ0.append(self.gZread)
        self.mX0.append(self.mXread)
        self.mY0.append(self.mYread)
        self.mZ0.append(self.mZread)
        
    def Euler2Q(self, phi, theta, psi):
        sinPhi = np.sin(phi/2)
        cosPhi = np.cos(phi/2)
        sinTheta = np.sin(theta/2)
        cosTheta = np.cos(theta/2)
        sinPsi = np.sin(psi/2)
        cosPsi = np.cos(psi/2)
        q1 = cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi
        q2 = sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi
        q3 = cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi
        q4 = cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi
        return q1,q2,q3,q4  
      
    def calcCal(self):
        self.a0[0] = np.mean(np.array(self.aX0))
        self.a0[1] = np.mean(np.array(self.aY0))
        self.a0[2] = np.mean(np.array(self.aZ0))
        self.g0[0] = np.mean(np.array(self.gX0))
        self.g0[1] = np.mean(np.array(self.gY0))
        self.g0[2] = np.mean(np.array(self.gZ0))
        self.m0[0] = np.mean(np.array(self.mX0))
        self.m0[1] = np.mean(np.array(self.mY0))
        self.m0[2] = np.mean(np.array(self.mZ0))
        self.velGlobal[0].append(0.0)
        self.velGlobal[1].append(0.0)
        self.velGlobal[2].append(0.0)
        self.inicio = True
        ###################################
        self.a0[3] = math.sqrt(self.a0[0]*self.a0[0] + self.a0[1]*self.a0[1] + self.a0[2]*self.a0[2])
            
        # self.theta0  =  np.arctan2(self.a0[1], self.a0[2])
        # self.phi0 = np.arctan2(-self.a0[0], math.sqrt(self.a0[1]*self.a0[1] + self.a0[2]*self.a0[2]))
        self.psi0 = 0.0
        self.theta0  =  np.arctan2(self.aXread, math.sqrt(self.aYread**2 + self.aZread**2))
        self.phi0 = np.arctan2(self.aYread, math.sqrt(self.aXread**2 + self.aZread**2))

        q1,q2,q3,q4 = self.Euler2Q(self.phi0,self.theta0,self.psi0)
        normQ = np.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        self.quaternion[0].append(q1/normQ)
        self.quaternion[1].append(q2/normQ)
        self.quaternion[2].append(q3/normQ)
        self.quaternion[3].append(q4/normQ)

    def quatProduct(self,Q,P):
        w = Q[0]*P[0] - Q[1]*P[1] - Q[2]*P[2] - Q[3]*P[3]
        x = Q[0]*P[1] + Q[1]*P[0] + Q[2]*P[3] - Q[3]*P[2]
        y = Q[0]*P[2] - Q[1]*P[3] + Q[2]*P[0] + Q[3]*P[1]
        z = Q[0]*P[3] + Q[1]*P[2] - Q[2]*P[1] + Q[3]*P[0]
        Qf= np.array([w,x,y,z])
        return Qf
    
    def Q2Euler(self,q1,q2,q3,q4):
        argPhi1 = 2*(q3*q4 + q1*q2)
        argPhi2 = 1 - 2*(q2*q2 + q3*q3)
        argTheta = 2*(q1*q3-q2*q4)
        argPsi1 = 2*(q2*q3 + q1*q4)
        argPsi2 = 1 - 2*(q3*q3 + q4*q4)
        #if abs(argTheta) > 1: 
        #    argTheta = argTheta/abs(argTheta)
        phi = np.arctan2(argPhi1, argPhi2)
        if np.abs(argTheta) >= 1:
            theta = np.sign(argTheta) * np.pi / 2
        else:
            theta = np.arcsin(argTheta)
        psi = np.arctan2(argPsi1, argPsi2)
        return phi,theta,psi
    
    def rotate_vector_inverted(self,vector,Q):
        #quat rotation matrix
        RQt = np.array([[Q[0]**2+Q[1]**2-Q[2]**2-Q[3]**2, 2*(Q[1]*Q[2]-Q[0]*Q[3]), 2*(Q[1]*Q[3]+Q[0]*Q[2])],
                        [2*(Q[1]*Q[2]+Q[0]*Q[3]),Q[0]**2-Q[1]**2+Q[2]**2-Q[3]**2, 2*(Q[3]*Q[2]-Q[0]*Q[1]) ],
                        [2*(Q[1]*Q[3]-Q[0]*Q[2]), 2*(Q[3]*Q[2]+Q[0]*Q[1]), Q[0]**2-Q[1]**2-Q[2]**2+Q[3]**2]])
        #inverse rotation
        RQt_T = RQt.T

        #transformation
        inverse_rotated_vector = np.dot(RQt_T, vector)

        return inverse_rotated_vector
    
    def rotate_vector(self,vector,Q):
        #quat rotation matrix
        RQt = np.array([[Q[0]**2+Q[1]**2-Q[2]**2-Q[3]**2, 2*(Q[1]*Q[2]-Q[0]*Q[3]), 2*(Q[1]*Q[3]+Q[0]*Q[2])],
                [2*(Q[1]*Q[2]+Q[0]*Q[3]),Q[0]**2-Q[1]**2+Q[2]**2-Q[3]**2, 2*(Q[3]*Q[2]-Q[0]*Q[1])],
                [2*(Q[1]*Q[3]-Q[0]*Q[2]), 2*(Q[3]*Q[2]+Q[0]*Q[1]), Q[0]**2-Q[1]**2-Q[2]**2+Q[3]**2]])

        #transformation
        rotated_vector = np.dot(RQt, vector)

        return rotated_vector
    
    def adaptative_alpha(self,acc_v,max_alpha = 0.2,G= 9.80665):  
        #vector aceleración, alpha máximo, gravedad

        L_norm= np.linalg.norm(acc_v)  #||a||
        e_m= abs(L_norm-G)/G #error de la medida

        #función de ganancia
        if e_m < 0.1:
            gain_factor = 1
        elif e_m >= 0.1 and e_m <0.2:
            gain_factor = 1-10*(e_m-0.1)
        elif e_m >= 0.2:
            gain_factor = 0

        #alpha adaptativo   
        alpha= max_alpha * gain_factor

        return alpha
    
    def update(self):
        aX = self.aXread
        aY = self.aYread
        aZ = self.aZread
        acc = [aX, aY, aZ]
        p = self.gXread - self.g0[0]
        q = self.gYread - self.g0[1]
        r = self.gZread - self.g0[2]
        mX = self.mXread - self.m0[0]
        mY = self.mYread - self.m0[1]
        mZ = self.mZread - self.m0[2]
        mag = [mX, mY, mZ]
        mag = mag/np.linalg.norm(mag) # normalización del vector magnetómetro

        #Predicción cuaternión G->L

        # se accede al último cuaternión
        qt_1 = np.array([self.quaternion[0][-1], self.quaternion[1][-1], self.quaternion[2][-1],self.quaternion[3][-1]])
        # se calcula la forma matricial de la velocidad angular
        omegaW = [[0, p, q , r], [-p, 0, r, -q], [-q, -r, 0, p], [-r, q, -p, 0]]
        # se calcula el diferencial del cuaternión
        dQ = np.dot(omegaW, qt_1)
        # se calcula el cuaternión predicho
        Qt = qt_1 + dQ*self.dt
        

        #Predicción del vector gravedad
        gGp = self.rotate_vector_inverted(acc, Qt) #medidas de acelerómetro en el marco global     
        norm_gGp = gGp/np.linalg.norm(gGp) # normalización del cuaternión gravedad predicha

        #Componentes del cuaternión corrector de aceleración despejadas
        dQacc0 = math.sqrt((norm_gGp[2]+1)/2)
        dQacc1 = -norm_gGp[1]/(math.sqrt(2*(norm_gGp[2]+1)))
        dQacc2 = norm_gGp[0]/math.sqrt(2*(norm_gGp[2]+1))
        dQacc3 = 0

        #Cuaternión corrector de aceleración
        dQacc = np.array([dQacc0, dQacc1, dQacc2, dQacc3]) 
        
        #Filtro de corrección
        threshold = 0.9
        alpha = self.adaptative_alpha(acc, max_alpha= 0.3, G= 9.80665)
        qI = np.array([1, 0, 0, 0])


        #LERP
        if dQacc0 > threshold:
            dQacc_m = (1-alpha)*qI +alpha*dQacc
            dQacc_est = dQacc_m / np.linalg.norm(dQacc_m)
        
        #SLERP
        else:
            dot_product = np.dot(qI, dQacc)
            norm_I = np.linalg.norm(qI)
            norm_dQacc = np.linalg.norm(dQacc)

            cos_theta = dot_product / (norm_I * norm_dQacc)
            theta = np.arccos(cos_theta)
            sin_theta = np.sin(theta)
            sin_a_theta= np.sin(alpha*theta)
            sin_1a_theta = np.sin((1-alpha)*theta)
            
            dQacc_m = (sin_1a_theta/sin_theta)*qI + (sin_a_theta/sin_theta)*dQacc
            dQacc_est = dQacc_m / np.linalg.norm(dQacc_m)

        #Quaternion corregido por acc
        Qacc = self.quatProduct(Qt,dQacc_est)
        w, x, y, z = Qacc #en caso de desactivar el magnetómetro, aqui se toma el quaternion final

        #Magnetómetro
        #---------------------------------------------------------------------------
        if self.mag_cal == True:
            
            Eh= self.rotate_vector(mag,Qacc)

            Eb0 = 0
            Eb1 = np.sqrt(Eh[0]**2+Eh[1]**2)
            Eb2 = 0
            Eb3 = Eh[2]
            Eb = np.array([Eb0,Eb1,Eb2,Eb3])

            #Predicción de la orientación con corrector magnético
            mean_g0= (self.g0[0] + self.g0[1] + self.g0[2])/ 3
            beta = np.sqrt(3/4)* mean_g0
            #beta = 1
            Qmag= self.MadgwickMagPrediction(qt_1, dQ, beta, Eb, mag)
            # filtro complementario
            gamma = 0.2
            Qf= gamma*Qmag + (1-gamma)*Qacc
            w, x, y, z = Qf        
        #---------------------------------------------------------------------------
            #Quaternion final normalizado
        Qf= np.array([w,x,y,z])
        Qfnorm= np.linalg.norm(Qf)
        w = w/Qfnorm
        x = x/Qfnorm
        y = y/Qfnorm
        z = z/Qfnorm 
        
        #Guardar Quaternion
        self.quaternion[0].append(w)
        self.quaternion[1].append(x)
        self.quaternion[2].append(y)
        self.quaternion[3].append(z)
    
        thetaF, phiF, psiF = self.Q2Euler(w,x,y,z)
        dt_var = (self.time_cicle[0] - self.time_zero)/1000
        # print(float(round(thetaF*180/np.pi,2)),'; ',float(round(phiF*180/np.pi,2)),'; ', float(round(psiF*180/np.pi,2)))
        self.dt_save.append(dt_var)
        # self.Datawrite_sync[0] = float(w)
        # self.Datawrite_sync[1] = float(x)
        # self.Datawrite_sync[2] = float(y)
        # self.Datawrite_sync[3] = float(z)
        self.Datawrite_sync[0] = float(round(w,3))
        self.Datawrite_sync[1] = float(round(x,3))
        self.Datawrite_sync[2] = float(round(y,3))
        self.Datawrite_sync[3] = float(round(z,3))

        self.plotTime.append(self.time_cicle[0])
        self.Roll.append(thetaF)
        self.Pitch.append(phiF)
        self.Yaw.append(psiF)

        self.channel.basic_publish(
                exchange='topic_Proc',
                routing_key=self.key_binding,
                body=json.dumps(self.Datawrite_sync)
            )
        
    def callback(self,ch, method, propierties, body):
        datos = json.loads(body) #carga los datos del mensaje en formato lista
        if self.conteo <= 250: 
            self.initialCalibration(datos) #guarda los datos de las primeras 249 lecturas
            self.conteo += 1   
            if self.conteo == 250:
                self.calcCal() #al llegar a 250 lecturas, calcula los offsets
                self.conteo +=1
                print("""
#####################################
#####################################
Calibración finalizada. Puede moverse.
#####################################
#####################################
                      """)
        elif self.inicio: #si ya se ha hecho la calibración, comienza el proceso normal de lectura
            self.readData(datos)
            self.update()
            self.conteo+=1
            # print('Conteo:',self.conteo)  
        else: #después, comienza el proceso normal de lectura
            self.readData(datos)
            self.update()
            self.conteo+=1
            
            # print('Conteo:',self.conteo)
    def stop(self):
        if self.process and self.process.is_alive():
            print("Deteniendo el proceso IMU...")
            self.stop_event.set()  # Señal para terminar el bucle
            self.process.join()  # Espera a que el proceso termine
            print("Proceso IMU detenido.")
        else:
            print("El proceso IMU no estaba en ejecución.")
            try:
                self.channel.close()
            except Exception as e:
                print(f"{e}")
            try:
                self.conn.close()
            except Exception as e:
                print(f"{e}")
        
    def run(self):
        self.channel.basic_consume(
        queue=self.queue_name, on_message_callback=self.callback, auto_ack=True)
        self.channel.start_consuming()       