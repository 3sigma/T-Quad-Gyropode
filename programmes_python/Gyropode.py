#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de contrôle du robot T-Quad en mode gyropode
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.1.1 - 23/02/2017
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino import *

# Imports Généraux
import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Gestion de l'IMU
from mpu9250 import MPU9250


# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega(hostname = hostname)

# Moteurs
Nmoy = 1

omegaArriereDroit = 0.
codeurArriereDroitDeltaPos = 0
codeurArriereDroitDeltaPosPrec = 0

omegaArriereGauche = 0.
codeurArriereGaucheDeltaPos = 0
codeurArriereGaucheDeltaPosPrec = 0

# Tension effectivement appliquée
commandeArriereDroit = 0.
commandeArriereGauche = 0.

# Saturations
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

# Asservissements
Kpvx = -19.6 # gain proportionnel pour l'asservissement de vitesse longitudinale
Kivx = -8.3 # gain intégral pour l'asservissement de vitesse longitudinale
Kpomega = -5. * 0.6 # gain proportionnel pour l'asservissement de verticalité
Kiomega = -32. # gain intégral pour l'asservissement de verticalité
Kpxi = 0.33 # gain proportionnel pour l'asservissement de rotation
Kixi = 12.6 # gain intégral pour l'asservissement de rotation
commande_vx = 0. # commande pour l'asservissement de vitesse longitudinale
commande_omega = 0. # commande pour l'asservissement de verticalité
P_vx = 0. # action proportionnelle pour l'asservissement de vitesse longitudinale
I_vx = 0. # action intégrale pour l'asservissement de vitesse longitudinale
P_omega = 0. # action proportionnelle pour l'asservissement de verticalité
I_omega = 0. # action intégrale pour l'asservissement de verticalité
P_xi = 0. # action proportionnelle pour l'asservissement de rotation
I_xi = 0. # action intégrale pour l'asservissement de rotation
thetaest = 0. # angle d'inclinaison estimé par le filtre complémentaire
tau = 0.98 # paramètre du filtre complémentaire
vxmes = 0.
xiodo = 0.
ximes = 0.

# Paramètres mécaniques
R = 0.03 # Rayon d'une roue
W = 0.15 # Ecart entre les roues

# Variables utilisées pour les données reçues
vxref = 0.
omegaref = 0.
thetaref = 0.
xiref = 0.
source_ximes = 0

# Déclarations pour la gestion des modes "asservissement actif" et "chute"
startedTQuad = False
juststarted = True
signe_ax = 0
thetamesprec = 0
omegaprec = 0
gzprec = 0

# Timeout de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

T0 = time.time()
dt = 0.01
i = 0
tprec = time.time()
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

idecimLectureTension = 0
decimLectureTension = 6000
decimErreurLectureTension = 100

idecimDistance = 0
decimDistance = 20
# Mesure de la tension de la batterie
# On la contraint à être supérieure à 7V, pour éviter une division par
# zéro en cas de problème quelconque
lectureTensionOK = False
tensionAlim = 7.4
while not lectureTensionOK:
    try:
        tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
        lectureTensionOK = True
    except:
        print("Erreur lecture tension")

# Capteur de distance
pulse_start = 0
pulse_end = 0
pulse_duration = 0
last_pulse_duration = 0
distance = 0
distancePrec = 0
distanceFiltre = 0
tauFiltreDistance = 0.03

if (hostname == "pcduino"):
    trig = 10
    echo = 13
    # Initialisation
    pinMode(trig, OUTPUT)
    pinMode(echo, INPUT)
elif (hostname == "raspberrypi"):
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    trig = 3 # GPIO22
    echo = 23
    # Initialisation
    pinMode(trig, OUTPUT)
    GPIO.setup(echo,GPIO.IN)
else:
    # pcDuino par défaut
    trig = 10
    echo = 13
    # Initialisation
    pinMode(trig, OUTPUT)
    pinMode(echo, INPUT)

# Initialisation de l'IMU
gz = 0.
if (hostname == "pcduino"):
    I2CBUS = 2
elif (hostname == "raspberrypi"):
    I2CBUS = 1
else:
    # pcDuino par défaut
    I2CBUS = 2
    
initIMU_OK = False
while not initIMU_OK:
    try:
        imu = MPU9250(i2cbus=I2CBUS, address=0x69)
        initIMU_OK = True
    except:
        print("Erreur init IMU")

        
#--- setup --- 
def setup():
    global signe_ax
    
    # Initialisation des moteurs
    CommandeMoteurs(0, 0, 0, 0)
    
    # Initialisation du capteur de distance
    digitalWrite(trig, LOW)
    print "Attente du capteur de distance"
    time.sleep(2)
    
    digitalWrite(trig, HIGH)
    time.sleep(0.00001)
    digitalWrite(trig, LOW)
    
    # On démarre seulement quand le gyropode dépasse la verticale
    signe_ax = 1

    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i, T0
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --

def CalculVitesse():
    global omegaArriereDroit, omegaArriereGauche, timeLastReceived, timeout, timedOut, \
        tdebut, codeurArriereDroitDeltaPos, codeurArriereGaucheDeltaPos, \
        commandeArriereDroit, commandeArriereGauche, \
        codeurArriereDroitDeltaPosPrec, codeurArriereGaucheDeltaPosPrec, tprec, \
        idecimLectureTension, decimLectureTension, decimErreurLectureTension, tensionAlim, \
        pulse_start, pulse_end, pulse_duration, last_pulse_duration, distance, idecimDistance, decimDistance, distancePrec, \
        distanceFiltre, tauFiltreDistance, imu, gz, R, W, vxmes, ximes, vxref, xiref, source_ximes, hostname, source_ximes, xiodo, \
        thetamesprec, omegaprec, thetaest, tau, omegaref, thetaref, startedTQuad, juststarted, signe_ax, I_vx, I_omega, I_xi, gzprec
    
    tdebut = time.time()
        
    # Tant que l'asservissement de verticalité n'est pas activé
    while not startedTQuad:

        tprec = time.time() - dt
                
        # Le robot vient de chuter
        if juststarted:

            # On réinitialise l'asservissement et on entre dans la boucle d'attente de redressement
            juststarted = False

            # Réinitilisation de l'asservissement
            I_vx = 0.
            I_omega = 0.
            I_xi = 0.
            thetaest = 0.
            thetamesprec = 0.
            omegaprec = 0.
            gzprec = 0.
            codeurArriereDroitDeltaPosPrec = 0
            codeurArriereGaucheDeltaPosPrec = 0

        # Mesure de la pesanteur
        accel = imu.readAccel()
        ax = accel['z'] * 9.81
        
        # Si l'accélération change de signe, cela signifie que le robot a été redressé pour le faire démarrer
        if (ax * signe_ax) <0:

            # Donc, on démarre
            startedTQuad = True
            print "Demarrage"

            # Les consignes reçues sont initilisées à zéro
            vxref = 0.
            xiref = 0.


        else:
            # On garde tout à zéro
            I_vx = 0.
            I_omega = 0.
            I_xi = 0.
            thetaest = 0.
            thetamesprec = 0.
            omegaprec = 0.
            gzprec = 0.
            codeurArriereDroitDeltaPosPrec = 0
            codeurArriereGaucheDeltaPosPrec = 0
            
            
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:
        codeursArriereDeltaPos = mega.read_codeursArriereDeltaPos()
        codeurArriereDroitDeltaPos = codeursArriereDeltaPos[0]
        codeurArriereGaucheDeltaPos = codeursArriereDeltaPos[1]
        
        # Suppression de mesures aberrantes
        if (abs(codeurArriereDroitDeltaPos - codeurArriereDroitDeltaPosPrec) > 10) or (abs(codeurArriereGaucheDeltaPos - codeurArriereGaucheDeltaPosPrec) > 10):
            codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
            codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
    
        codeurArriereDroitDeltaPosPrec = codeurArriereDroitDeltaPos
        codeurArriereGaucheDeltaPosPrec = codeurArriereGaucheDeltaPos
    except:
        #print "Error getting data"
        codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
        codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
        
    omegaArriereDroit = -2 * ((2 * 3.141592 * codeurArriereDroitDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaArriereGauche = 2 * ((2 * 3.141592 * codeurArriereGaucheDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
        
    # Mesures des vitesses longitudinales et de rotation
    vxmes = (omegaArriereDroit + omegaArriereGauche) * R / 2
    xiodo = (omegaArriereDroit - omegaArriereGauche) * R / W
        
    # Lecture des mesures de l'IMU
    try:
        accel = imu.readAccel()
        gyro = imu.readGyro()
        
        # Acceleration longitudinale
        thetames = accel['z']
        # Vitesse de chute mesurée par le gyroscope convertie en rad/s
        omega = gyro['y'] * math.pi / 180.
        # Vitesse de rotation autour de la verticale mesurée par le gyroscope convertie en rad/s
        gz = gyro['x'] * math.pi / 180
        
        thetamesprec = thetames
        omegaprec = omega
        gzprec = gz
    except:
        #print("Erreur lecture IMU")
        thetames = thetamesprec
        omega = omegaprec
        gz = gzprec
        pass

    # Vitesse de rotation, en fonction de la source utilisée
    if (source_ximes == 1):
        ximes = gz
    else:
        ximes = xiodo
    
    dt2 = time.time() - tprec
    tprec = time.time()
    
    # Estimation de l'angle à partir de l'accélération horizontale (filtre complémentaire)
    thetaest = (tau*dt2*omega + dt2*thetames + tau*thetaest)/(dt2+tau)

    # Test pour savoir si le gyropode est toujours debout
    if startedTQuad:

        # Si l'angle d'inclinaison n'est pas trop prononcé
        if abs(thetames) < 0.75:
            en = 1.

        # Sinon, cela signifie que le robot est tombé
        else:
            # On désactive l'asservissement
            en = 0.
            startedTQuad = False
            juststarted = True

            # Réinitialisation pour détecter un futur redressement
            if thetames > 0:
                signe_ax = 1
            else:
                signe_ax = -1

    # Si le robot n'a pas encore démarré, l'asservissement est désactivé
    else:
        en = 0.
        

    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout and not timedOut:
        timedOut = True
        
    if timedOut:
        vxref = 0.
        xiref = 0.
        
    # Application de la consigne lue
    vxref = vxref * en
    xiref = xiref * en

    # Définition des entrées de la fonction d'asservissement
    vxmes = vxmes * en
    omegames = omega * en
    ximes = ximes * en
    
    # Calcul du PI sur vx
    
    # Terme proportionnel (la transformation de la commande par retour d'état en PI
    # conduit à une référence nulle, d'où le 0.*vxref)
    P_vx = Kpvx * (0. * vxref - vxmes)

    # Calcul de la commande
    commande_vx = P_vx + I_vx

    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_vx = I_vx + Kivx * dt2 * (vxref - vxmes)

    # Fin Calcul du PI sur vx

    # Calcul du PI sur omega
    # Terme proportionnel
    P_omega = Kpomega * (omegaref - omegames)
    
    # Terme "intégral" (c'est en fait un terme proportionnel sur theta)
    I_omega = Kiomega * (thetaref - thetaest)

    # Calcul de la commande
    commande_omega = P_omega + I_omega

    # Fin Calcul du PI sur omega
    
    # Calcul du PI sur xi
    
    # Terme proportionnel
    P_xi = Kpxi * (xiref - ximes)

    # Calcul de la commande
    commande_xi = P_xi + I_xi


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_xi = I_xi + Kixi * dt2 * (xiref - ximes)

    # Fin Calcul du PI sur xi
    
    commandeLongi = commande_vx + commande_omega
    
    
    # Transformation des commandes longitudinales et de rotation en tension moteurs
    commandeArriereDroit = -(commandeLongi + commande_xi) * en # Tension négative pour faire tourner positivement ce moteur
    commandeArriereGauche = (commandeLongi - commande_xi) * en
    
    CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, 0, 0)
    
    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        try:
            tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
            idecimLectureTension = 0
        except:
            # On recommence la lecture dans decimErreurLectureTension * dt
            idecimLectureTension = idecimLectureTension - decimErreurLectureTension
            #print("Erreur lecture tension dans Loop")
    else:
        idecimLectureTension = idecimLectureTension + 1
    
    
    # Calcul de la distance mesurée par le capteur ultrason
    # On fait ce calcul après l'affichage pour savoir combien de temps
    # il reste pour ne pas perturber la boucle
    if idecimDistance >= decimDistance:            
        idecimDistance = 0
        digitalWrite(trig, HIGH)
        time.sleep(0.00001)
        digitalWrite(trig, LOW)
        
        if (hostname == "pcduino"):
            pulse_duration = 0
            while (digitalRead(echo) == 0) and (time.time() - tdebut < dt):
                pulse_start = time.time()

            while (digitalRead(echo) == 1) and (pulse_duration < 0.01166) and (time.time() - tdebut < dt):
                pulse_end = time.time()
                last_pulse_duration = pulse_duration
                pulse_duration = pulse_end - pulse_start
                
        elif (hostname == "raspberrypi"):
            pulse_duration = 0
            while (GPIO.input(echo) == 0) and (time.time() - tdebut < dt):
                pulse_start = time.time()

            while (GPIO.input(echo) == 1) and (pulse_duration < 0.01166) and (time.time() - tdebut < dt):
                pulse_end = time.time()
                last_pulse_duration = pulse_duration
                pulse_duration = pulse_end - pulse_start
                
        else:
            pulse_duration = 0
            while (digitalRead(echo) == 0) and (time.time() - tdebut < dt):
                pulse_start = time.time()

            while (digitalRead(echo) == 1) and (pulse_duration < 0.01166) and (time.time() - tdebut < dt):
                pulse_end = time.time()
                last_pulse_duration = pulse_duration
                pulse_duration = pulse_end - pulse_start
            
                                
        distance = last_pulse_duration * 17150
        distance = round(distance, 0)
        # Filtre sur la distance
        distanceFiltre = (dt * distance + tauFiltreDistance * distancePrec) / (dt + tauFiltreDistance)
        distancePrec = distanceFiltre
    else:
        idecimDistance = idecimDistance + 1

        
    #print time.time() - tdebut

    
def PID(iMoteur, omegaref, omega, Kp, Ki, Kd, Tf, umax, umin, dt2):
    global I_x, D_x, yprec
    
    # Calcul du PID
    # Paramètres intermédiaires
    Ti = Ki/(Kp+0.01)
    if (Kd>0): # Si PID
        ad = Tf/(Tf+dt2)
        bd = Kd/(Tf+dt2)
        Td = Kp/Kd
        Tt = sqrt(Ti*Td)
    else: # Si PI
        ad = 0
        bd = 0
        Td = 0
        Tt = 0.5*Ti
    
    br = dt2/(Tt+0.01)

    # Calcul de la commande avant saturation
        
    # Terme proportionnel
    P_x = Kp * (omegaref - omega)

    # Terme dérivé
    D_x[iMoteur] = ad * D_x[iMoteur] - bd * (omega - yprec[iMoteur])

    # Calcul de la commande avant saturation
    commande_avant_sat = P_x + I_x[iMoteur] + D_x[iMoteur]

    # Application de la saturation sur la commande
    if (commande_avant_sat > umax):
        commande = umax
    elif (commande_avant_sat < umin):
        commande = umin
    else:
        commande = commande_avant_sat
    
    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_x[iMoteur] = I_x[iMoteur] + Ki * dt2 * (omegaref - omega) + br * (commande - commande_avant_sat)
    
    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprec[iMoteur] = omega
    
    return commande


def CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    global tensionAlim
    
    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionArriereDroit = commandeArriereDroit
    tensionArriereGauche = commandeArriereGauche
    tensionAvantDroit = commandeAvantDroit
    tensionAvantGauche = commandeAvantGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_ArriereDroit = int(255 * tensionArriereDroit / tensionAlim)
    tension_int_ArriereGauche = int(255 * tensionArriereGauche / tensionAlim)
    tension_int_AvantDroit = int(255 * tensionAvantDroit / tensionAlim)
    tension_int_AvantGauche = int(255 * tensionAvantGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_ArriereDroit > 255):
        tension_int_ArriereDroit = 255

    if (tension_int_ArriereDroit < -255):
        tension_int_ArriereDroit = -255

    if (tension_int_ArriereGauche > 255):
        tension_int_ArriereGauche = 255

    if (tension_int_ArriereGauche < -255):
        tension_int_ArriereGauche = -255

    if (tension_int_AvantDroit > 255):
        tension_int_AvantDroit = 255

    if (tension_int_AvantDroit < -255):
        tension_int_AvantDroit = -255

    if (tension_int_AvantGauche > 255):
        tension_int_AvantGauche = 255

    if (tension_int_AvantGauche < -255):
        tension_int_AvantGauche = -255

    # Commande PWM
    try:
        mega.moteursArriere(tension_int_ArriereDroit, tension_int_ArriereGauche)
        mega.moteursAvant(tension_int_AvantDroit, tension_int_AvantGauche)
        mega.moteursCRC(tension_int_ArriereDroit + tension_int_ArriereGauche, tension_int_AvantDroit + tension_int_AvantGauche)
    except:
        pass
        #print "Erreur moteurs"

    
def emitData():
    global tprec
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    #delay(5000)
    tprec = time.time()
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 100)
        self.callback.start()
    

    def on_message(self, message):
        global vxref, xiref, source_ximes, timeLastReceived, timedOut
            
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
        
        if jsonMessage.get('vxref') != None:
            vxref = float(jsonMessage.get('vxref')) / 100.
        if jsonMessage.get('xiref') != None:
            xiref = float(jsonMessage.get('xiref')) * math.pi / 180.
        if jsonMessage.get('source_ximes') != None:
            # Choix de la source de la vitesse de rotation mesurée: 1: gyro, 0: vitesse des roues
            source_ximes = int(jsonMessage.get('source_ximes'))
            
        if not socketOK:
            vxref = 0.
            xiref = 0.

        
    def on_close(self):
        global socketOK, vxref, xiref
        print 'connection closed...'
        socketOK = False
        vxref = 0.
        xiref = 0.

    def sendToSocket(self):
        global socketOK, vxmes, omegaArriereDroit, omegaArriereGauche, gz, xiodo, vxref, xiref
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), \
                                'consigne_vx':("%.2f" % vxref), \
                                'consigne_xi':("%.2f" % xiref), \
                                'vxmes':("%.2f" % vxmes), \
                                'xiodo':("%.2f" % xiodo), \
                                'distance':("%d" % distance), \
                                'distanceFiltre':("%d" % distanceFiltre), \
                                'gz':("%.2f" % gz), \
                                'Raw':("%.2f" % tcourant) \
                                + "," + ("%.2f" % vxmes) \
                                + "," + ("%.2f" % ximes) \
                                + "," + ("%d" % distance) \
                                + "," + ("%d" % distanceFiltre) \
                                + "," + ("%.2f" % gz) \
                                })
                                
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    global vxref, xiref
    print 'Sortie du programme'
    vxref = 0.
    xiref = 0.
    CommandeMoteurs(0, 0, 0, 0)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


