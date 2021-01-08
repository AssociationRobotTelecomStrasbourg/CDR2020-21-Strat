/* 
Code de test de la carte-mère du robot pour la CDR2020
  Pour le microcontrôleur en charge de la stratégie
 */
#include <Arduino.h>
#include <Wire.h>
#include "pins_strt.h"
#include "VL53L0X.h"
#include "VL53Network.h"
#include "strat.h"

#include "commFrame.h"
#include "commKeywords.h"
#include "commMvt.h"
#define _debug

//variable gestion IHM-STRAT
uint16_t msgIHM = 0;
bool attenteGo = false;
bool goMatch = false;

//Réseau lidars fixes
VL53Network reseauLidar(4);
uint8_t N_ok_reseau;

//Gestion batterie
#define BATT_RATIO 3 //Rapport pont diviseur de tension du senseur batterie
float Vbat = 0;
uint16_t Vbat_raw;


commFrame movt(FRAMESTARTER);
bool tested;


#define matchDuration 100000 //100 * 1000 milliseconds
bool firstLoop = true;
uint32_t goTime; //point de départ du compteur de temps écoulé

/*-------------------------------------*/
//---------------SETUP-----------------//
/*-------------------------------------*/

void setup() {
  // put your setup code here, to run once:

  //Initialisation des ports série
  Movt.begin(9600);
  SerialIHM.begin(9600);
  SerialUSB.begin(9600);

  //Test Leds de debug

  ledsInit(); //initialisation des LEDs de debug

  //clignotement test des leds
  ledsBlinkSeq();

  pinMode(13,OUTPUT); //built-in led init
  #ifdef debug
    while(!Serial){
      //Attente de l'ouverture de la liaison série USB
      delay(50);
    }
  #endif

  /*Envoi des messages de réveil à MOVT et à IHM*/
  delay(250);//On laisse le temps aux autres de se reveiller
  //Message de reveil envoyé aux autres microcontrôleurs, qui répondent si message reçu correctement
  Movt.write(0x7E);
  SerialIHM.write(0x7E);

  /*Traitement des données reçues*/
  delay(100);//Petit temps de traitement

  int msgTestMOVT = Movt.read(); //Lecture de la réponse de MOVT

  #ifdef debug
      Serial.print("MSG T.MOVT: ");
      Serial.println(msgTestMOVT,HEX);
  #endif

  //Verification de la réponse de MOVT (doit etre 0x7E, sinon il y a un problème)
  if(msgTestMOVT == 0x7E){
    SerialIHM.write(0x43);//Signifie à l'IHM que liaison avec Teensy Movt est OK
    digitalWrite(13,HIGH);
    #ifdef debug
      Serial.println("Liaison T.MOVT OK");
    #endif
  }

  else{
    SerialIHM.write(0xEE); //Signifie à l'IHM PB avec Teensy Movt
    #ifdef debug
      Serial.println("PB Liaison T.MOVT");
    #endif
  }

  //Test tension batterie
  pinMode(VBATT,INPUT);
  Vbat = readVbatt(Vbat_raw);

  #ifdef debug
    Serial.println(Vbat_raw, DEC);
    Serial.print("Tension batterie : ");
    Serial.print(Vbat);
    Serial.println(" V");
  #endif

  //Envoi de la tension batterie, avec division du 16 bits en 2 x 8 bits
  SerialIHM.write(Vbat_raw>>8);
  SerialIHM.write(Vbat_raw & 0x00FF);
  digitalWrite(13,HIGH);
  delay(100);

  //INIT capteur reflectivité
  pinMode(ReflecSens,INPUT);

  #ifdef debug 
    Serial.print("Lecture reflectance: ");
    Serial.println(analogRead(ReflecSens));
  #endif
  /* Initialisation LIDARs fixes*/
  
  Wire1.begin();
  //Ecriture de la liste des broches XSHUT
  //
  //Paramètrage du réseau
  N_ok_reseau = lidarNetworkInit(reseauLidar,DEFAULT_TIMEOUT,100);
  SerialIHM.write(N_ok_reseau); //envoi de l'information à l'IHM

  #ifdef debug
    //Affichage infos du reseau lidar pour debug
    Serial.print("Nb lidars dans reseau: ");
    Serial.println(reseauLidar.getN());
    Serial.print("XSHUTS: ");
    uint8_t x1,x2,x3,x4;
    reseauLidar.getXSHUT(x1,x2,x3,x4);
    Serial.print(x1);
    Serial.print("|");
    Serial.print(x2);
    Serial.print("|");
    Serial.print(x3);
    Serial.print("|");
    Serial.print(x4);
    Serial.println("|");
    Serial.print("Nb lidars initialisés: ");
    Serial.println(N_ok_reseau);
  #endif

  //PWM LIDAR
  pinMode(PWM_LIDAR,OUTPUT);
  //Essai de plusieurs vitesses de rotation pour le Lidar rotatif
  LidarSpeedTestSequence(); //2100 millisecondes de delai
  LidarSetSpeed(125);
  //LIDAR UART
  SerialLidar.begin(9600);

  //UART DEBUG
  Serial4.begin(9600);

  #ifdef debug
    Serial.print("Valeur GoSensor: ");
    Serial.println(digitalRead(GO_SENSOR));
  #endif
  
  ledsWrite(0,0);

  #ifdef debug
    Serial.println("Debut attente du match");
    delay(250);
  #endif
  
  //Code d'attente du début du match
  while(!goMatch){
    #ifdef debug
      Serial.println("WHILE ATTENTE");
      delay(250);
    #endif
    while(SerialIHM.available()<2 and !attenteGo){
      //Attente message d'attente match de la part de l'IHM
      delay(100);
      #ifdef debug
        Serial.println("Attente msg...");
      #endif
    }
    //Lecture du message
    if(SerialIHM.available()==2){
      msgIHM = SerialIHM.read();
      #ifdef debug
        Serial.print("MSG RECU: ");
        Serial.println(msgIHM,HEX);
      #endif
    }

    /*
    Si le message correspond à la consigne d'attente, et qu'on attend pas déjà:
      Explication : "Attente" = Attente que la tirette de départ soit tiree par membre de l'équipe
      msgIHM == 0x66 -> equipe a lancé le match via IHM
    En pratique, l'equipe utilise l'IHM, puis tire sur la tirette
    */

    if(!attenteGo && msgIHM==0x66){ //Debut attente tirette
      attenteGo = true;
      //digitalWrite(LED1, HIGH);
      ledsWrite(1,0);
      #ifdef debug
        Serial.println("Attente GO!");
      #endif
    }
    #ifdef debug
      if(attenteGo){
        Serial.print("Valeur GO_SENSOR:");
        Serial.println(analogRead(GO_SENSOR));
      }
    #endif

    //Si on attend et que la match a pas commencé, lecture de l'état de la tirette (analogRead(Go_SENSOR)~=1023)
    if(attenteGo && !goMatch && analogRead(GO_SENSOR)>1000){
      goMatch = true; //Sortie de la boucle d'attente
      SerialIHM.write(0x56); //Confirmation reception à l'IHM
      Movt.write(0x56); //Transmission d'info à movt
      ledsWrite(0,1);
      #ifdef debug
        Serial.println("Tirette ouverte, match GO!");
      #endif
      //LEDs indiquent le départ
      ledsWrite(1,1);
    }
  }
  //Fin code d'attente
}

void loop() {

  #ifdef debug
      Serial.println("LOOP!");
      delay(250);
  #endif

  //Initialisation valeur de depart du chrono
  if(firstLoop){
    goTime = millis();
    firstLoop = false;
  }

  //fct d'arret du robot a la fin du match
  if(millis()-goTime >= matchDuration){
    goMatch = false;
    //TODO: arret des moteurs
  }
  
  //Code actions du match
    //Temporaire
  if(!tested){ //test transmission de consigne
    tested = true;
    moveXY(500,500,movt);
  }

  //actions du match
  while(goMatch){
    

    //Lecture des capteurs
      //LIDARs fixes
      //LIDARs mobiles
      //position, via movt
        //requestCoords
        //Serial.read
          //if reponse :: OK -> attente de lecture
          //if reponse :: ERR -> renvoi de trame (n essais)
          //timeout...
    //Fct de décision

    //Envoi trame d'ordre a processeur

    //Lecture buffer serie strat/mvt
      //si Ack: OK (acquittement de mvt)
      //si Err: renvoi de la trame, n essais (Erreur sur trame detectee par movt)
      //si timeout: renvoi de la trame, n essais (Aucun reponse de movt)
  }
}

