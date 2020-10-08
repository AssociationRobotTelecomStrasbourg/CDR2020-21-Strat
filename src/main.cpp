/* 
Code de test de la carte-mère du robot pour la CDR2020
  Pour le microcontrôleur en charge de la stratégie
 */
#include <Arduino.h>
#include <Wire.h>
#include "pins_strt.h"
#include "VL53L0X.h"
#include "VL53Network.h"

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

void setup() {
  // put your setup code here, to run once:

  //Initialisation des ports série
  Movt.begin(9600);
  SerialIHM.begin(9600);
  SerialUSB.begin(9600);

  //Test Leds de debug
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,HIGH);
  delay(500);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,HIGH);
  delay(500);
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,LOW);
  delay(500);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  
  pinMode(13,OUTPUT); //built-in led init
  #ifdef debug
    while(!Serial){
      //Attente de l'ouverture de la liaison série USB
      delay(50);
    }
  #endif

  /*Envoi des messages de réveil à MOVT et à IHM*/
  delay(250);//On laisse le temps aux autres de se reveiller
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
  Vbat_raw = analogRead(VBATT);
  Vbat = (map(Vbat_raw,0,1023,0,3.3)*4.0)*1.044; //1.044 : facteur de calibration

  #ifdef debug
    Serial.println(Vbat_raw, DEC);
    Serial.print("Tension batterie : ");
    Serial.print(Vbat);
    Serial.println(" V");
  #endif 
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
  u_int8_t* list = new u_int8_t[4];
  list[0] = XSHUTa;
  list[1] = XSHUTb;
  list[2] = XSHUTc;
  list[3] = XSHUTd;
  //
  //Paramètrage du réseau
  reseauLidar.setTimeout(500);
  reseauLidar.set_xshut(list);
  reseauLidar.initNetwork();
  reseauLidar.startContinuous(100);
  N_ok_reseau = reseauLidar.getN_OK(); //Contient le nombre de capteurs initialises avec succes
  SerialIHM.write(N_ok_reseau); //envoi de l'information à l'IHM

  #ifdef debug
    Serial.print("Nb lidars initialisés: ");
    Serial.println(N_ok_reseau);
  #endif

  //PWM LIDAR
  //Essai de plusieurs vitesses de rotation pour le Lidar rotatif
  pinMode(PWM_LIDAR,OUTPUT);
  analogWrite(PWM_LIDAR,255);
  delay(700);
  analogWrite(PWM_LIDAR,125);
  delay(700);
  analogWrite(PWM_LIDAR,0);
  delay(700);
  analogWrite(PWM_LIDAR,125);
  delay(700);



  //UART DEBUG
  Serial4.begin(9600);

  SerialLidar.begin(9600);

  #ifdef debug
    Serial.print("Valeur GoSensor: ");
    Serial.println(digitalRead(GO_SENSOR));
  #endif
  
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

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

    //Si le message correspond à la consigne d'attente, et qu'on attend pas déjà:
    ////Explication : "Attente" = Attente que la tirette de départ soit tiree par membre de l'équipe
    if(!attenteGo && msgIHM==0x66){
      attenteGo = true;
      digitalWrite(LED1, HIGH);
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
      digitalWrite(LED2, HIGH);
      #ifdef debug
        Serial.println("Tirette ouverte, match GO!");
      #endif
    }
  }
  //Fin code d'attente
}

void loop() {
  #ifdef debug
      Serial.println("LOOP!");
      delay(250);
  #endif
  /* //Test mesures lidar
  delay(500);
  uint16_t* measures = reseauLidar.readRangeContinuousMillimeters();
  Serial.print("Mesures lidars: ");
  Serial.print(measures[0]);
  Serial.print("|");
  Serial.print(measures[1]);
  Serial.print("|");
  Serial.print(measures[2]);
  Serial.print("|");
  Serial.print(measures[3]);
  Serial.println("|"); 
  */


 



  //Code actions du match
  while(goMatch){
    //actions du match
    /* digitalWrite(LED2, HIGH);
    digitalWrite(LED1, HIGH); */
  }
}

