import math
import numpy as np
import random as r
import matplotlib.pyplot as plt
import csv

class slidingRMS:
    def __init__(self, window):
        self.window = window
        self.sumatory = 0
        self.windowlist = []
    def process(self, raw_data): 
        self.windowlist.append(raw_data)
        if raw_data != 0:
            self.sumatory += (raw_data/abs(raw_data))*raw_data**2

        if len(self.windowlist) > self.window:
            data2pop= self.windowlist.pop(0)
            
            if data2pop != 0:
                self.sumatory -= (data2pop/abs(data2pop))*data2pop**2
        if self.sumatory != 0:
            RMS_value = float(np.sqrt(abs(self.sumatory)/len(self.windowlist))*(self.sumatory/abs(self.sumatory)))
        else:
            RMS_value = 0
        return RMS_value
    
def noisySignal_maker(offset, noiserange, length):
    signal = []
    for i in range(round(length*0.4)):
        signal.append(offset)
    for j in range(round(length*0.2)):
        signal.append(float(offset+(offset/abs(offset))*20*abs(np.sin(j/40)+j*0.02)))
    for k in range(round(length*0.2)):
        signal.append(offset)
    for l in range(len(signal)):
        signal[l]= signal[l]+(80/signal[l])*r.randrange(round(-abs((noiserange)*offset/100)),round(abs((noiserange)*offset/100)))
    return signal     
def readcsv(archivo): 
    datos = []
    with open(archivo, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        # Saltar la primera línea (header)
        next(reader, None)
        # Recoger cada fila como vector
        for row in reader:
            datos.append(row)
    return datos

def normfunction100(data_deg, rangeMin, rangeMax):
    data = data_deg[rangeMin:rangeMax]
    N = len(data)
    x_old = np.linspace(0, 99, N)   # dominio original (0–99 %)
    x_new = np.arange(100)          # destino: 100 instancias

    # Para ángulos en grados: desenrollar - interpolar - enrollar
    rad = np.deg2rad(data)
    u   = np.unwrap(rad)
    u_i = np.interp(x_new, x_old, u)
    y   = (u_i + np.pi) % (2*np.pi) - np.pi
    return np.rad2deg(y)            # 100 puntos normalizados 

def filter_Test(window):
    noisesignal = readcsv('C:/Users/Usuario/Desktop/output.csv') 

    for i in range(len(noisesignal)):
        noisesignal[i]= float(noisesignal[i][0]) + r.uniform(-0.005, 0.005)
        random_p = r.randint(0, 1000)
        if random_p < 15:
            noisesignal[i] = noisesignal[i] + r.uniform(-0.3, 0.3)

    processedsignal= []
    filter1 = slidingRMS(window)
    for i in range(len(noisesignal)):
        processedsignal.append(filter1.process(float(noisesignal[i])))

    sample_interval = 0.01
    n = len(processedsignal[1000:2500])
    t = np.arange(n) * sample_interval
    plt.figure()
    plt.plot(t, processedsignal[1000:2500],  label='Signal')
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Value")
    plt.title("Processed Signal")
    plt.ylim(min(processedsignal) - 0.5, max(processedsignal) + 0.5)
    plt.legend()
    plt.grid(True)

    sample_interval= 0.01
    n = len(noisesignal)
    t = np.arange(n) * sample_interval
    plt.figure()
    plt.plot(t, noisesignal,  label='Signal')
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Value")
    plt.title("Noise Signal")
    plt.ylim(min(processedsignal) - 0.5, max(processedsignal) + 0.5)
    plt.legend()
    plt.grid(True)
    normfunction100(processedsignal, 1000, 2500)    
    

filter_Test(100)
plt.show()