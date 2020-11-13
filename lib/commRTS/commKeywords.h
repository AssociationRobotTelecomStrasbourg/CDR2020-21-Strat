/* Ce fichier contient les mots-clés permettant le codage / décodage des trames de communication
entre le processeur stratégie et le processeur mouvement*/

#ifndef COMMKEYWORDS_H
#define COMMKEYWORDS_H

#define COMMSERIAL Serial2 //Changer ceci pour configurer le port série à utiliser avec la classe comm

#define FRAMESTARTER 0x18

#define ID_goTo_XY 0x55
#define ID_goTo_Rtheta 0x56
#define ID_STOP 0x66
#define ID_SPEED_CONF 0x70
#define ID_ACCEL_CONF 0x73
#define ID_SERVO 0x57

#endif