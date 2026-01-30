//**********PREPROCESSEURS_DEBUT**********        //SECTION POUR DÉFINIR DES MACROS ET DES LIBRAIRES
//---------------IMPORTANTS---------------
#define PI                          3.14159       //DEFINIR PI
#define MOTOR_WHEEL_RADIUS          0.017         //DEFINIR LA LARGEUR DES ROUES EN METRES
//DEFINIR LE PERIMÈTRE DES ROUES
#define MOTOR_WHEEL_PERIMETER       (float) ( 2.0 * ((float) PI) * ((float) MOTOR_WHEEL_RADIUS) )
#define MOTORS_SPAN                 0.09          //DEFINIR LA DISTANCE ENTRE LES ROUES EN METRES (NECESSAIRE POUR LA ROTATION)
//DEFINIR LE PERIMÈTRE DU CERCLE CREE PAR LA DISTANCE ENTRE LES ROUES LORS DE LA ROTATION SUR PLACE
#define MOTORS_SPAN_PERIMETER       (float) ( 2.0 * ((float) PI) * ((float) ( ((float) MOTORS_SPAN) / 2.0)) )

#define TILE_LENGTH                 0.13          //DEFINIR LA LONGUEUR D'UNE TUILE DU TERRAIN EN METRES

#define TIMER_TPS                   20            //DEFINIR LE NOMBRE DE TRAITEMENTS DU PROGRAMME DU ROBOT CHAQUE SECONDE
//DEFINIR L'INTERVALLE DE TEMPS ENTRE CHAQUE TRAITEMENT DU PROGRAMME DU ROBOT EN SECONDES
#define TIMER_INTERVAL_S            (float) ( 1.0 / ((float) TIMER_TPS) )
//DEFINIR L'INTERVALLE DE TEMPS ENTRE CHAQUE TRAITEMENT DU PROGRAMME DU ROBOT EN MILLISECONDES
#define TIMER_INTERVAL_MS           (unsigned long) ( 1000UL / ((unsigned long) TIMER_TPS) )

//-----------------MOTEUR-----------------
#define MOTOR_GEAR_RATIO            60
#define MOTOR_PPR_SHAFT             6
#define MOTOR_PPR_WHEEL             MOTOR_PPR_SHAFT * MOTOR_GEAR_RATIO

#define MOTOR_MOVEMENT_PWM          255           //DEFINIR LA PUISSANCE DE L'EFFORT POUR LE MOUVEMENT
#define MOTOR_ROTATION_PWM          255           //DEFINIR LA PUISSANCE DE L'EFFORT POUR LA ROTATION
#define MOTOR_MOVEMENT_PWM_MIN      200           //DEFINIR LA PUISSANCE MINIMALE DE L'EFFORT POUR LE MOUVEMENT
//DEFINIR LE NOMBRE D'IMPULSIONS NÉCESSAIRE POUR UN DEPLACEMENT DE 13CM
#define MOTOR_MOVEMENT_PULSE        (int) ( ( ((float) TILE_LENGTH) / ((float) MOTOR_WHEEL_PERIMETER) ) * ((float) MOTOR_PPR_WHEEL) )
//DEFINIR LE NOMBRE D'IMPULSIONS NÉCESSAIRE POUR UNE ROTATION DE 90°
#define MOTOR_ROTATION_PULSE        (int) ( ( ( ((float) MOTORS_SPAN_PERIMETER) / 4.0) / ((float) MOTOR_WHEEL_PERIMETER)) * ((float) MOTOR_PPR_WHEEL) )
//DEFINIR LE NOMBRE D'IMPULSIONS PAR INTERVALLE DE TEMPS DESIREE LORS DU DEPLACEMENT EN PULSE/DT DEPUIS 5cm/s
#define MOTOR_MOVEMENT_SPEED        (int) ( (0.06 / ((float) MOTOR_WHEEL_PERIMETER)) * ((float) MOTOR_PPR_WHEEL) * ((float) TIMER_INTERVAL_S) )
//DEFINIR LE NOMBRE D'IMPULSIONS PAR INTERVALLE DE TEMPS DESIREE LORS DE LA ROTATION EN PULSE/DT DEPUIS 3.125cm/s
#define MOTOR_ROTATION_SPEED        (int) ( (0.04 / ((float) MOTOR_WHEEL_PERIMETER)) * ((float) MOTOR_PPR_WHEEL) * ((float) (TIMER_INTERVAL_S)) )

#define MOTOR_KP_LR                 5.0
#define MOTOR_KI_LR                 0.1
#define MOTOR_KD_LR                 0.1
#define MOTOR_KP_HE                 0.1
#define MOTOR_KI_HE                 0.0
#define MOTOR_KD_HE                 0.1

#define MOTOR_IN1                   12//2         //LE L293D EST UTILISE POUR CONTROLER LES DEUX MOTEURS SIMPLES 
#define MOTOR_IN2                   4             //IN1, IN2, IN3 et IN4 PERMETTENT DE CONTROLER LA DIRECTION DE ROTATION
#define MOTOR_ENA                   5             //EN IMPOSANT UN DIFFERENCE DE POTENTIEL ENTRE EN1 <-> EN2, EN3 <-> EN4
#define MOTOR_IN3                   13//3         //UNE TECHNIQUE APPELEE H-BRIDGE 
#define MOTOR_IN4                   7             //ENA ET ENB CONTROLENT LA VITESSE DES MOTEURS AVEC LA TECHNOLOGIE PWM
#define MOTOR_ENB                   6             //PWM ETANT PULSE WIDTH MODULATION
#define ENCODER_L_C1                3             //EMET UN SIGNAL LORS DE LA ROTATION DE LA ROUE GAUCHE
#define ENCODER_R_C1                2             //EMET UN SIGNAL LORS DE LA ROTATION DE LA ROUE DROITE
//--------------TELECOMMANDE--------------
#define DECODE_NEC                                //UTILISER UNE LIBRAIRIE POUR TRAITER LES SIGNAUX DE LA TÉLECOMMANDE
#include <IRremote.hpp>                           //LE PWM DES PINS 3 ET 11 SERONT INUTILISABLES (TIMER 2)
#define REMOTE                      8             //DÉFINIT LE PIN POUR LA TÉLÉCOMMANDE
//-----------------LIGHT-----------------
#define LIGHT                       11            //DEFINIT LE PIN POUR LA DEL TEMOIN
//-----------ULTRASONIC SENSORS-----------
#include <NewPing.h>                              //UTILISER UNE LIBRAIRIE POUR TRAITER LES SIGNAUX DU HC-SR04 (TIMER 2)
#define TRIGGER                     9             //DEFINIT LE PIN POUR LE TRIGGER DU HC-SR04
#define ECHO                        10            //DEFINIT LE PIN POUR L'ECHO DU HC-SR04
#define ECHO_DISTANCE_MAX           100           //DEFINIT LA DISTANCE MAXIMALE DETECTEE PAR LE PROGRAMME EN CM
#define OBSTACLE_DISTANCE_THRESHOLD 5             //DEFINIT LA DISTANCE MAXIMALE ENTRE LE HC-SR04 ET UN OBJET POUR ÊTRE UN OBSTACLE EN CM
NewPing sonar(TRIGGER, ECHO, ECHO_DISTANCE_MAX);  //INITIALISER LA LIBRAIRIE
//--------------INTELLIGENCE--------------
#define PATH_LIST_MAX 120                         //DEFINIR LA LONGUEUR DE LA LISTE DECRIVANT LE CHEMIN DU ROBOT
//DEFINIR UNE FONCTION POUR CONVERTIR DEUX OCTETS EN UN CHIFFRE 16 BITS ENTIER
#define BYTES_TO_INT(x, y)          ((x) + (((int)(y)) << 8))
//DEFINIR UNE FONCTION QUI EXTRAIT L'OCTET INFERIEUR D'UN CHIFFRE 16 BITS 
#define INT_TO_BYTE_LOWER(x)        ((byte)((x) & 0xFF))
//DEFINIR UNE FONCTION QUI EXTRAIT L'OCTET SUPERIEUR D'UN CHIFFRE 16 BITS
#define INT_TO_BYTE_UPPER(x)        ((byte)(( (x)>>8 ) & 0xFF))
//DEFINIR UNE FONCTION QUI PERMET DE CONTEXTUALISER LES FONCTIONS D'AU DESSUS
#define GET_VISITED_X(x)            INT_TO_BYTE_LOWER(x)
//LE CONTEXTE ETANT NOTRE GRILLE DE CASES VISITEES int visited[][] POUR LE PARCOURS EN LARGEUR
#define GET_VISITED_Y(x)            INT_TO_BYTE_UPPER(x)
#define SET_VISITED(x, y)           BYTES_TO_INT(x, y)
#define BFS_UNVISITED               0xFFFF        //DEFINIT UNE CONSTANTE PERMETTANT DE MARQUER UNE CASE COMME ETANT PAS VISITEE
//DEFINIR UN FONCTION QUI PERMET DE TRANSFORMER TROIS CHIFFRES DANS UNE LISTE {1,2,3} EN L'ENTIER QU'ILS REPRESENTERAIENT 123
//UTILISE PAR LA LISTE remoteCommand
#define DIGITS_TO_INT(x, y, z)      (x*100) + (y*10) + z
#define TABLES_MAX                  255           //DEFINIR LE NOMBRE DE TABLES MAXIMALE DANS UN RESTAURANT
#define TIMER_SERVING               TIMER_TPS*10  //DEFINIR LA QUANTITIE DE TEMPS QUE LE ROBOT SERVIRA SES CLIENTS EN UNITÉS D'INTERVALLE DE TEMPS
#define TIMER_BLOCKED               TIMER_TPS*5   //DEFINIR LA QUANTITIE DE TEMPS QUE LE ROBOT ATTNEND POUR RETENTER D'AVANCER APRES L'OBSTACLE EN UNITÉS D'INTERVALLE DE TEMPS
//DEFINIR UNE FONCTION POUR AVOIR LE NIEME BIT D'UN CHIFFRE
#define GET_NTH_BIT(v, n)           ( (( (int) v ) & (1 << ( (int) n) )) >> ( (int) n) )
//CES FONCTIONS SONT UTILISEES AVEC UN CHRONOMETRE AVANCANT INFINIMENT
#define GET_4TH_BIT(v)              GET_NTH_BIT(v, 3)
//POUR AVOIR UN MOTIF DE 0 ET DE 1 AU FIL DU TEMPS
#define GET_6TH_BIT(v)              GET_NTH_BIT(v, 5)
//UTILISEE PAR LE DEL
#define GET_7TH_BIT(v)              GET_NTH_BIT(v, 6)
#define GET_2ND_BIT(v)              GET_NTH_BIT(v, 1)
#define GET_3RD_BIT(v)              GET_NTH_BIT(v, 2)
#define GET_4TH_BIT(v)              GET_NTH_BIT(v, 3)
#define GET_5TH_BIT(v)              GET_NTH_BIT(v, 4)
//***********PREPROCESSEURS_FIN***********

//******FONCTIONS_ET_VARIABLES_DEBUT******        //SECTION POUR DÉFINIR DES CONSTANTES/VARIABLES GLOBALES
//-----------------MOTEUR-----------------
enum motorStates {                                //DÉFINIT TOUS LES ÉTATS POSSIBLES DES MOTEURS
  INERT,                                          //ON FAIT RIEN
  FORWARDS,                                       //ON AVANCE
  BACKWARDS,                                      //ON RECULE
  LEFT,                                           //ON PART A GAUCHE
  RIGHT                                           //ON PART A DROITE
};
motorStates motorState;                           //VARIABLE GLOBAL DÉFINISSANT L'ÉTAT DES MOTEURS
byte motorRightPWM;
byte motorLeftPWM;
volatile int motorRightPulse;
volatile int motorLeftPulse;
int motorRightPulsePrevious;
int motorLeftPulsePrevious;
int motorSpeedTarget;
int motorPulseTarget;
float motorRightI;
float motorLeftI;
short motorRightErrorPrevious;
short motorLeftErrorPrevious;
float motorHeadingI;
short motorHeadingErrorPrevious;

void move_start(bool forwards = 1) {              //FONCTION QUI FAIT AVANCER OU RECULER LE ROBOT
  digitalWrite(MOTOR_IN1, forwards ? LOW : HIGH);
  digitalWrite(MOTOR_IN2, forwards ? HIGH : LOW);
  analogWrite(MOTOR_ENA, MOTOR_MOVEMENT_PWM);

  digitalWrite(MOTOR_IN3, forwards ? LOW : HIGH);
  digitalWrite(MOTOR_IN4, forwards ? HIGH : LOW);
  analogWrite(MOTOR_ENB, MOTOR_MOVEMENT_PWM);
}
void move_stop() {                                //FONCTION QUI ARRÊTE TOUT MOUVEMENT DU ROBOT
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
  
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENB, 0);
}
void turn_start(bool right = 1) {                 //FONCTION QUI FAIT TOURNER LE ROBOT A DROITE OU A GAUCHE 90°
  digitalWrite(MOTOR_IN1, right ? LOW : HIGH);
  digitalWrite(MOTOR_IN2, right ? HIGH : LOW);
  analogWrite(MOTOR_ENA, MOTOR_ROTATION_PWM);
  
  digitalWrite(MOTOR_IN3, right ? HIGH : LOW);
  digitalWrite(MOTOR_IN4, right ? LOW : HIGH);
  analogWrite(MOTOR_ENB, MOTOR_ROTATION_PWM);
}
void motor_update() {                             //FONCTION QUI ACTIONNE LES MOTEURS SELON LA VARIABLE motorState
  noInterrupts();
  motorRightPulse = 0;
  motorLeftPulse = 0;
  interrupts();
  motorRightPulsePrevious = 0;
  motorLeftPulsePrevious = 0;
  motorRightI = 0;
  motorLeftI = 0;
  motorRightErrorPrevious = 0;
  motorLeftErrorPrevious = 0;
  motorHeadingI = 0;
  motorHeadingErrorPrevious = 0;
  switch (motorState) {
    case motorStates::FORWARDS:
      motorRightPWM = MOTOR_MOVEMENT_PWM;
      motorLeftPWM = MOTOR_MOVEMENT_PWM;
      motorSpeedTarget = MOTOR_MOVEMENT_SPEED;
      motorPulseTarget = MOTOR_MOVEMENT_PULSE;
      move_start(1);
      break;
    case motorStates::BACKWARDS:
      motorRightPWM = MOTOR_MOVEMENT_PWM;
      motorLeftPWM = MOTOR_MOVEMENT_PWM;
      motorSpeedTarget = MOTOR_MOVEMENT_SPEED;
      motorPulseTarget = MOTOR_MOVEMENT_PULSE;
      move_start(0);
      break;
    case motorStates::LEFT:
      motorRightPWM = MOTOR_ROTATION_PWM;
      motorLeftPWM = MOTOR_ROTATION_PWM;
      motorSpeedTarget = MOTOR_ROTATION_SPEED;
      motorPulseTarget = MOTOR_ROTATION_PULSE;
      turn_start(0);
      break;
    case motorStates::RIGHT:
      motorRightPWM = MOTOR_ROTATION_PWM;
      motorLeftPWM = MOTOR_ROTATION_PWM;
      motorSpeedTarget = MOTOR_ROTATION_SPEED;
      motorPulseTarget = MOTOR_ROTATION_PULSE;
      turn_start(1);
      break;
    default:
      move_stop();
      motorRightPWM = 0;
      motorLeftPWM = 0;
      motorSpeedTarget = 0;
      motorPulseTarget = 0;
      break;
  }
}
void interruptMotorEncoderRight () {
  motorRightPulse++;
}
void interruptMotorEncoderLeft () {
motorLeftPulse++;
}
//--------------TELECOMMANDE--------------
enum remoteStates {                               //DÉFINIT TOUS LES ÉTATS POSSIBLES DE LA TÉLÉCOMMANDE
  NONE    = 0,                                    //CHAQUE VALEUR EST LE CODE DU BOUTON CORRESPONDANT DE LA TELECOMMANDE
  OFF     = 69,                                   //NONE CORRESPOND A L'ETAT OÙ AUCUN BOUTON A ETE APPUYE
  VOLUP   = 70,
  FUNC    = 71,
  BACK    = 68,
  PLAY    = 64,
  FORWARD = 67,
  DOWN    = 7,
  VOLDOWN = 21,
  UP      = 9,
  ZERO    = 22,
  EQ      = 25,
  ST      = 13,
  ONE     = 12,
  TWO     = 24,
  THREE   = 94,
  FOUR    = 8,
  FIVE    = 28,
  SIX     = 90,
  SEVEN   = 66,
  EIGHT   = 82,
  NINE    = 74
};
remoteStates remoteState;                         //VARIABLE GLOBAL DÉFINISSANT L'ÉTAT DE LA TÉLÉCOMMANDE
byte remoteCommand[3] = {0, 0, 0};                //LISTE STOCKANT LA TABLE CIBLE DE L'UTILISATEUR
byte remoteCommandIndex = 0;                      //VARIABLE SERVANT À ECRIRE LA COMMANDE CHIFFRE PAR CHIFFRE
int detect_remote() {                             //FONCTION QUI DETECTE DES COMMANDES POTENTIELS ET LE STOCKE DANS remoteState
  if (IrReceiver.decode()) {                      //SI L'ON A RECU UNE COMMANDE
    uint16_t recieved = 0;
    if(IrReceiver.decodedIRData.protocol == NEC) {//ET QU'ELLE PROVIENT DU PROTOCOLE NEC
      recieved = IrReceiver.decodedIRData.command;//ON DECODE LA COMMANDE
    }
    IrReceiver.resume();                          //ON CONTINUE DE CHERCHER D'AUTRES SIGNAUX
    return (int)recieved;                         //ET ON CONVERTIT LE RESULTAT EN ENTIER ET ON LE STOCKE DANS remoteState
  }
  return remoteStates::NONE;                      //LA VALEUR PAR DEFAUT RENVOYE SERA NONE POUR DIRE QUE L'ON A RIEN RECU
}
//-----------ULTRASONIC SENSORS-----------
byte distance;                                    //VARIABLE QUI STOCKE LA DISTANCE ENTRE LE ROBOT ET L'ESPACE DEVANT LUI SELON LE HC-SR04
//--------------INTELLIGENCE--------------
enum robotStates {                                //DÉFINIT TOUS LES ÉTATS POSSIBLES DU SERVEUR
  STANDBY,                                        //LE ROBOT ATTEND D'ETRE COMMANDE
  TRAVEL,                                         //LE ROBOT SE DEPLACE
  BLOCKED,                                        //LE ROBOT EST BLOQUE
  SERVE                                           //LE ROBOT SERT DES CLIENTS
};
robotStates robotState;                           //VARIABLE GLOBAL DÉFINISSANT L'ÉTAT DU SERVEUR
unsigned int tickGlobal = 0;                      //CHRONOMETRE INTERNE DU ROBOT INCREMENTANT INFINIMENT
unsigned int tickLocal = 0;                       //CHRONOMETRE INTERNE DU ROBOT DECREMENTANT INFINIMENT REINITIALISABLE SELON L'UTILITE
unsigned long previousTime = 0;                   //VARIABLE STOCKANT LE TEMPS DE L'ANCIEN TOUR DU BOUCLE loop EN MILLISECONDES
unsigned long currentTime;                        //VARIABLE STOCKANT LE TEMPS DU NOUVEAU TOUR DU BOUCLE loop EN MILLISECONDES

enum gridStates {                                 //DÉFINIT LES CASES DU RESTAURANT (SAUF TABLES)
  FLOOR = 0,                                      //TOUS LES 0 SERONT TOUJOURS LE SOL
  WALL = 1                                        //TOUS LES 1 SERONT TOUJOURS LE MUR
};
const byte GRID_WIDTH = 10;                        //NOMBRE DE COLONNES DANS LA CARTE
const byte GRID_HEIGHT = 10;                       //NOMBRE DE LIGNES DANS LA CARTE
byte restaurant[GRID_HEIGHT][GRID_WIDTH] = {      //CARTE POUR LA NAVIGATION DU ROBOT
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},                 //0       = SOL
  {1, 1, 1, 1, 7, 0, 1, 1, 1, 1},                 //1       = MUR/OBSTACLE
  {1, 1, 0, 1, 1, 0, 2, 1, 2, 1},                 //[3-255] = TABLES
  {1, 1, 0, 0, 0, 0, 1, 1, 1, 1},
  {1, 6, 0, 1, 1, 0, 3, 1, 3, 1},
  {1, 1, 0, 0, 0, 0, 1, 1, 1, 1},
  {1, 1, 1, 5, 4, 1, 1, 1, 1, 1},
  {1, 1, 1, 5, 4, 1, 1, 1, 1, 1},
  {1, 1, 1, 5, 4, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};
const byte baseX = 2;                             //DEFINIT LES COORDONNEES DE LA BASE DU ROBOT
const byte baseY = 2;                             //OU IL COMMENCERA TOUJOURS SON SERVICE
byte targetX;                                     //DEFINIT LES COORDONNEES DE LA CIBLE DU ROBOT
byte targetY;                                     //UTILISEE LORSQUE QUE LE ROBOT EST COMMANDE

byte path[PATH_LIST_MAX];                         //LISTE POUR DÉFINIR LE CHEMIN DU SERVEUR
byte pathIndex;                                   //INDEX DE LA LISTE POUR LA CREATION ET LE TRAITEMENT DU CHEMIN DU ROBOT
void clear_path() {                               //EFFACE LA LISTE DECRIVANT LE CHEMIN DU ROBOT (INUTILE?)
  for(unsigned int i = 0; i < PATH_LIST_MAX; i++) {//CETTE LISTE EST UNE LISTE D'INSTRUCTIONS SE TERMINANT AVEC INERT
    path[i] = motorStates::INERT;                 //DU MOMENT QUE L'ON ECRASE LA LISTE AVEC UN NOUVEL ENSEMBLE DE VALEURS DE LONGUEUR QUELCONQUE
  }                                               //QUI SE TERMINE AVEC UN INERT, CETTE FONCTION EST REDONDANT
}

enum robotAngles {                                //DÉFINIT TOUS LES ANGLES DU ROBOT
  NORTH,                                          //DANS LE SENS HORAIRE
  EAST,
  SOUTH,
  WEST
};
byte robotX = baseX;                              //VARIABLES GLOBALES DÉFINISSANT LES COORDONNEES DU ROBOT
byte robotY = baseY;                              //LE ROBOT COMMENCE TOUJOURS DEPUIS SA BASE
byte robotA = robotAngles::SOUTH;                 //VARIABLE GLOBAL DÉFINISSANT L'ANGLE DU ROBOT

//VARIABLE GLOBALE DÉFINISSANT LES CASES DEJA VISITEES PAR LE ROBOT
unsigned int visited[GRID_HEIGHT][GRID_WIDTH] = {0};
int queue[GRID_HEIGHT*GRID_WIDTH];                //LISTE GLOBALE AGISSANT COMME FILE
byte queueStart;                                  //LA LISTE SE COMPORTE COMME UNE FILE GRACE A CES DEUX VARIABLES
byte queueEnd;                                    //NECESSAIRE PORU LE PARCOUR EN LARGEUR
int neighborOffsets[4][2] = {                     //LISTE GLOBALE AIDANT A OBTENIR LES CASES ADJACENTES A UNE CASE DONNEE
  {0, -1},                                        //CASE DU NORD
  {1, 0},                                         //CASE DE L'EST
  {0, 1},                                         //CASE DU SUD 
  {-1, 0}                                         //CASE DE L'OUEST
};
byte retries;                                     //VARIABLE STOCKANT LE NOMBRE DE FOIS LE ROBOT ATTENDRA POUR L'OBSTACLE DE PARTIR AVANT DE LE CONTOURNER

bool is_queue_empty() {                           //FONCTION DISANT SI LA FILE EST VIDE
  return queueStart == queueEnd;
}
bool enqueue(int q) {                             //FONCTION ENFILANT UNE VALEUR q DONNEE
  queue[queueEnd++] = q;
  return queueEnd < GRID_HEIGHT*GRID_WIDTH;       //SI L'ON DEPASSE LA LONGUEUR DE LA FILE, PREVENIR
}
int dequeue() {                                   //FONCTION DEFILANT UNE VALEUR DE LA FILE
  return queue[queueStart++];
}
bool is_tile_IB(int x, int y) {                   //FONCTION DECLARANT SI UNE CASE EST DANS LES BORNES DU RESTAURANT
  return (x >= 0 && x < GRID_WIDTH) && (y >= 0 && y < GRID_HEIGHT);
}

bool search_maze(byte destX, byte destY) {        //FONCTION QUI PARCOURT LE RESTAURANT EN UTILISANT L'ALGORITHME DU PARCOURS EN LARGEUR
  for(byte y = 0; y < GRID_HEIGHT; y++) {         //ON REINITIALISE LA GRILLE visited
    for(byte x = 0; x < GRID_WIDTH; x++) {
      visited[y][x] = BFS_UNVISITED;
    }
  }
  
  queueStart = 0;                                 //ON REINITIALISE LA TETE ET LA QUEUE DE LA FILE
  queueEnd = 0;
  enqueue(SET_VISITED(robotX, robotY));           //ON ENFILE ET MARQUE LA CASE DE DEPART COMME ETANT VISITEE
  visited[robotY][robotX] = SET_VISITED(robotX, robotY);

  while(!is_queue_empty()) {                      //TANT QUE LA FILE N'EST PAS VIDE
    int nItem = dequeue();                        //ON DEFILE LA VALEUR LA PLUS VIELLE
    if (
      (GET_VISITED_X(nItem) == destX) && 
      (GET_VISITED_Y(nItem) == destY)
    ) {                                           //SI LA CASE DEFILEE EST LA DESTINATION
      return true;                                //ON A TERMINE
    }
    
    for(byte i = 0; i < 4; i++) {                 //SINON, ON ESSAYE D'ENFILER LES CASES ADJACENTES
      //ON TROUVE LES COORDONNES DE LA CASE ADJACENTE
      int nx = (GET_VISITED_X(nItem)+neighborOffsets[i][0]);
      //ET ON VERIFIE A LA SUITE
      int ny = (GET_VISITED_Y(nItem)+neighborOffsets[i][1]);
      
      if(!is_tile_IB(nx, ny)) { continue; }       //SI LE VOISIN EST DANS LES BORNES DU RESTAURANT
      if(
        (restaurant[ny][nx] != gridStates::FLOOR) && 
        !((nx == destX) && (ny == destY))
      ) { continue; }                             //SI IL EST LE SOL OU LA DESTINATION
      if(visited[ny][nx] != BFS_UNVISITED) { continue; }//EST CE QU'IL N'A PAS ETE VISITEE AUPARAVANT
      
      visited[ny][nx] = nItem;                    //SI OUI, ON MARQUE LA CASE COMME VISITEE, EN METTANT LE PARENT COMME VALEUR
      enqueue(BYTES_TO_INT(nx, ny));              //ET ON ENFILE LE VOISIN
    }
  }
  return false;                                   //SI L'ON ARRIVE PAS A CALCULER LE TRAJET, ON RENVOIE FALSE
}

//FONCTION QUI PERMET DE TROUVER L'ANGLE ENTRE DEUX CASES
int face_tile_from(byte stepFinalX, byte stepFinalY, byte stepInitialX, byte stepInitialY) {
  if(stepFinalX != stepInitialX) { return stepFinalX < stepInitialX ? robotAngles::WEST : robotAngles::EAST; }
  else                           { return stepFinalY < stepInitialY ? robotAngles::NORTH : robotAngles::SOUTH; }
}

//FONCTION QUI PERMET DE TROUVER LA ROTATION NECESSAIRE POUR SE DEPLACER D'UNE CASE A UNE AUTRE
void rotate_robot(byte stepFinalA, byte stepInitialA) {
  if(stepFinalA%2 == stepInitialA%2) {            //VERIFIE SI L'ON DOIT SE RETOURNER OU PAS TOURNER DU TOUT
    if(stepFinalA != stepInitialA) {              //PERMET DE VERIFIER SI L'ON DOIT SE RETOURNER
      path[pathIndex++] = motorStates::RIGHT;     //SI OUI, ON TOURNE A DROITE DEUX FOIS
      path[pathIndex++] = motorStates::RIGHT;     //SINON, IL N'EST PAS NECESSAIRE DE TOURNER DU TOUT
    }
  }
  else {                                          //VERIFIE SI L'ON DOIT TOURNER A DROITE OU A GAUCHE
    path[pathIndex++] = ( ((stepFinalA - stepInitialA + 6) % 4) == 3 ) ? motorStates::RIGHT : motorStates::LEFT;
  }
}

//FONCTION QUI PERMET DE DEFINIR LES INSTRUCTIONS AU MOTEURS POUR NAVIGUER VERS LA DESTINATION DEPUIS LA LISTE DU PARCOURS EN LARGEUR
void reconstruct_path(byte stepFinalX, byte stepFinalY) {
  pathIndex = 0;                                  //ON DEFINIT L'INDEX DE LA LISTE D'INSTRUCTIONS AU MOTEURS
  //NOUS CONNAISSONS LA DESTINATION ET LE DEPART, MAIS LE PARCOURS EN LARGEUR DONNE LE CHEMIN INVERSE POUR ALLER DU DEPART A LA DESTINATION
  //IL DONNE 'LE PARENT' DE LA CASE DONNE ALLANT VERS LE DEPART

  //ON RETROUVE LE PARENT DE LA CASE DE LA DESTINATION (LA CASE QUI PART VERS LE DEPART)
  //CECI SERA LA CASE OU LE ROBOT IRA VERITABLEMENT POUR SERVIR, MAIS IL FERA FACE A "L'ENFANT"
  byte stepInitialX = GET_VISITED_X(visited[stepFinalY][stepFinalX]);
  byte stepInitialY = GET_VISITED_Y(visited[stepFinalY][stepFinalX]);
  //ON CALCULE L'ANGLE QUE LE ROBOT DEVRA AVOIR POUR FAIRE FACE AUX CLIENTS APRES ETRE "ARRIVEE"
  byte stepInitialA = robotAngles::NORTH;
  byte stepFinalA = face_tile_from(stepFinalX, stepFinalY, stepInitialX, stepInitialY);
  while(pathIndex < PATH_LIST_MAX) {              //TANT QUE NOUS NE SOMMES PAS ARRIVEES AU DEPART, ET QUE LA LISTE N'EST PAS PLEINE
    stepFinalX = stepInitialX;                    //NOUS AVONS LES COORDONNES DESTINATION ET DEPART ADJACENTS CHAQUE TOUR DE BOUCLE 
    stepFinalY = stepInitialY;                    //CHAQUE TOUR DE BOUCLE VA CALCULER LES ACTIONS NECESSAIRES POUR ALLER DE LA DESTINATION AU DEPART
    //OBTENIR LA CASE PARENT DE LA CASE
    stepInitialX = GET_VISITED_X(visited[stepFinalY][stepFinalX]);
    stepInitialY = GET_VISITED_Y(visited[stepFinalY][stepFinalX]);
    if(
      (stepInitialX == stepFinalX) && 
      (stepInitialY == stepFinalY)
    ) { break; }                                  //SI NOUS SOMMES ARRIVEES A LA DESTINATION, ARRETER LA BOUCLE
    //SINON, TROUVER L'ANGLE QUE LE ROBOT AURAIT INITIALEMENT LORS DU DEPLACEMENT
    stepInitialA = face_tile_from(stepFinalX, stepFinalY, stepInitialX, stepInitialY);
    //TOURNER LE ROBOT DE SORTE A CE QUE LE ROBOT SOIT ORIENTE COMME IL LE SERAIT "INITIALEMENT" A LA CASE PARENT
    rotate_robot(stepFinalA, stepInitialA);
    
    path[pathIndex++] = motorStates::FORWARDS;    //'AVANCER' LE ROBOT
    stepFinalA = stepInitialA;                    //METTRE A JOUR L'ANGLE POUR LE PROCHAIN CALCUL
  }
  rotate_robot(stepInitialA, robotA);             //ARRIVE A LA FIN DE LA BOUCLE, IL FAUT ALIGNER LE ROBOT AU CHEMIN CALCULE
  pathIndex--;
  for(byte j = 0; j < pathIndex-j; j++) {         //NOUS AVONS PRECEDEMMENT CONSTRUIT LES INSTRUCTIONS DE DEPLACEMENT A L'ENVERS
    path[j] ^= path[pathIndex-j];                 //IL FAUT MAINTENANT INTERCHANGER LES VALEURS POUR LES REMETTRE A L'ENDROIT
    path[pathIndex-j] ^= path[j];
    path[j] ^= path[pathIndex-j];
  }
  path[++pathIndex] = motorStates::INERT;         //MARQUER LE CHEMIN PAR INERT A LA FIN POUR SIGNALER L'ARRIVEE
}
void print_path(byte *list) {                     //FONCTION DE DEBOGUAGE POUR AFFICHER LE CHEMIN CALCULE
  Serial.println("THE PATH IS AS SUCH :");
  for(byte i = 0; i < PATH_LIST_MAX; i++) {
    switch(path[i]) {
      case motorStates::INERT :
        Serial.println("INERT");
        return;
      case motorStates::FORWARDS :
        Serial.print("FORWARDS");
        break;
      case motorStates::BACKWARDS :
        Serial.print("BACKWARDS");
        break;
      case motorStates::LEFT :
        Serial.print("LEFT");
        break;
      case motorStates::RIGHT :
        Serial.print("RIGHT");
        break;
    }
    Serial.print(" - ");
  }                                               //SI ON ARRIVE A CETTE PHRASE, C'EST QUE LA LISTE DU CHEMIN EST PLEINE
  Serial.println("OH NO");                        //ET N'EST PAS PROPREMENT BORNEE PAR 'INERT' A LA FIN
}

int find_table(byte tableNumber) {                //FONCTION QUI DONNE LES COORDONNES D'UNE TABLE DEPUIS SON NUMERO DANS LA GRILLE restaurant
  for(byte y = 1; y < GRID_HEIGHT-1; y++) {
    for(byte x = 1; x < GRID_WIDTH-1; x++) {
      if(restaurant[y][x] == tableNumber) {
        return BYTES_TO_INT(x, y);
      }
    }
  }
  return 0;                                       //SI ON LE TROUVE PAS, ON RENVOIE 0
}

void handle_remote_signal() {                     //FONCTION QUI TRAITE LE SIGNAL DE LA TELECOMMANDE LORS DE L'ETAT "STANDBY"
  switch(remoteState) {
    case remoteStates::ZERO:                      //SI C'EST UN CHIFFRE, ON LE STOCKE DANS remoteCommand
      remoteCommand[remoteCommandIndex++] = 0;
      break;
    case remoteStates::ONE:
      remoteCommand[remoteCommandIndex++] = 1;
      break;
    case remoteStates::TWO:
      remoteCommand[remoteCommandIndex++] = 2;
      break;
    case remoteStates::THREE:
      remoteCommand[remoteCommandIndex++] = 3;
      break;
    case remoteStates::FOUR:
      remoteCommand[remoteCommandIndex++] = 4;
      break;
    case remoteStates::FIVE:
      remoteCommand[remoteCommandIndex++] = 5;
      break;
    case remoteStates::SIX:
      remoteCommand[remoteCommandIndex++] = 6;
      break;
    case remoteStates::SEVEN:
      remoteCommand[remoteCommandIndex++] = 7;
      break;
    case remoteStates::EIGHT:
      remoteCommand[remoteCommandIndex++] = 8;
      break;
    case remoteStates::NINE:
      remoteCommand[remoteCommandIndex++] = 9;
      break;
    case remoteStates::PLAY:                      //SI C'EST LE BOUTON PLAY, ON CHANGE L'ETAT DU ROBOT POUR QUE think_standby TRAITE LA COMMANDE
      robotState = robotStates::TRAVEL;
      break;
    case remoteStates::OFF:                       //SI C'EST LE BOUTON OFF, ON CHANGE L'ETAT DU ROBOT POUR QUE think_standby TRAITE LA COMMANDE
      robotState = robotStates::BLOCKED;
      break;
    default:
      break;
  }
  remoteCommandIndex = remoteCommandIndex%3;      //NOTRE COMMANDE NE PEUT QUE AVOIR TROIS VALEURS, DONC L'INDEX DOIT ÊTRE ENTRE 0 ET 2 INCLUS
}

void think_travel_start() {
  IrReceiver.stopTimer();                         //ON DESACTIVE LA TELECOMMANDE POUR LE MOMENT DONNE
  robotState = robotStates::TRAVEL;               //FAIRE PROMENER LE ROBOT
  if(!search_maze(targetX, targetY)) {            //SI ON NE TROUVE PAS DE CHEMIN VERS LA TABLE, ANNULER
    Serial.println("Error when pathing");
    robotState = robotStates::STANDBY;
    IrReceiver.restartTimer();
    return;
  }
  Serial.println("VALID!");
  reconstruct_path(targetX, targetY);             //SINON, ON CREE LE CHEMIN VERS LE ROBOT
  if(targetX == baseX && targetY == baseY) {
    path[pathIndex++] = motorStates::FORWARDS;    //LE ROBOT NE PEUT PAS JUSTE FAIRE FACE A LA CIBLE, COMME POUR LES TABLES
    path[pathIndex] = motorStates::INERT;         //DONC ON DOIT AJOUTER UNE COMMANDE OU LE ROBOT AVANCE
  }
  print_path(path);
  pathIndex = 0;                                  //ON REINITIALISE LES VARIABLES POUR L'ETAT "TRAVEL"
  tickLocal = 0;
  retries = 0;
  motorState = path[pathIndex++];
  motor_update();
}

void think_standby() {                            //FONCTION QUI GUIDE LE ROBOT DANS L'ETAT "STANDBY"
  //CE MOTIF DE CLIGNOTEMENT DESIGNERA QUE LE ROBOT ATTEND UNE NOUVELLE INSTRUCTION
  digitalWrite(LIGHT, (GET_5TH_BIT(tickGlobal)) ? HIGH : LOW );

  int newSignal = detect_remote();                //ON CHERCHE A VOIR SI LA TELECOMMANDE A ETE UTILISE
  //Serial.print(newSignal);Serial.print(" - ");Serial.println(remoteState);
  if(newSignal == remoteStates::NONE) {           //SI CE N'EST PAS LE CAS, ANNULER
    remoteState = remoteStates::NONE;
    return;
  }
  if(newSignal == remoteState) {                  //CECI EVITE QUE QUAND ON MAINTIENT LE BOUTON APPUYÉ LE PROGRAMME LE TRAITE PLUDSIEURS FOIS A LA SUITE
    return;
  }
  remoteState = newSignal;
  Serial.print("COMMAND : ");Serial.print(remoteState);Serial.print(" (");
  //ON TRAITE LA COMMANDE DE L'UTILISATEUR
  handle_remote_signal();
  Serial.print(remoteCommand[0]);Serial.print("|");Serial.print(remoteCommand[1]);Serial.print("|");Serial.print(remoteCommand[2]);
  Serial.print(") --> ");
  if(robotState == robotStates::BLOCKED) {      //SI LE BOUTON "OFF" A ETE APPUYE
    Serial.print("CLEAR");                      //ON REINITAILISE LA LISTE remoteCommand ET L'INDEX QUI VA AVEC
    remoteCommandIndex = 0;
    remoteCommand[0] = 0;
    remoteCommand[1] = 0;
    remoteCommand[2] = 0;
    robotState = robotStates::STANDBY;          //ON REMET LE ROBOT A L'ATTENTE
  }
  else if(robotState == robotStates::TRAVEL) {  //SI LE BOUTON "PLAY" A ETE APPUYE
    Serial.print("TARVEL? ");
    //ON RETROUVE LE NUMERO DE LA TABLE DU ROBOT
    unsigned int tableNumber = DIGITS_TO_INT(remoteCommand[0], remoteCommand[1], remoteCommand[2]) + 1;
    //SI ELLE ELLE N'EST PAS DANS LES BORNES, ANNULER
    if(tableNumber < 2 || tableNumber > TABLES_MAX) {
      robotState = robotStates::STANDBY;
      return;
    }
    //ON RETROUVE LES COORDONNES DE LA TABLE
    int tableCoordsXY = find_table(tableNumber);
    if(tableCoordsXY == 0) {                      //SI ELLES NE SONT PAS VALIDES, ANNULER
      robotState = robotStates::STANDBY;
      return;
    }
    targetX = GET_VISITED_X(tableCoordsXY);       //OBTENIR L'ABCISSE ET L'ORDONNEE DE LA TABLE
    targetY = GET_VISITED_Y(tableCoordsXY);       //ET LE DEFINIR EN TANT QUE CIBLE DU ROBOT
    think_travel_start();
  }
  Serial.println();
  //remoteCommandIndex %= 3;
}

void update_robot_after_action() {                //FONCTION QUI MET A JOUR LES COORDONNEES DU ROBOT APRES UN MOUVEMENT
  if(
      (motorState == motorStates::FORWARDS) ||
      (motorState == motorStates::BACKWARDS)
    ) {                                           //SI ON AVANCE OU ON RECULE
      if((robotA%2) == 0) {                       //ET QUE LE ROBOT EST ORIENTE AU NORD OU AU SUD
        robotY += (robotA == robotAngles::NORTH) ? -1 : +1;
      }
      else {                                      //OU QUE LE ROBOT EST ORIENTE A L'OUEST OU A L'EST
        robotX += (robotA == robotAngles::WEST) ? -1 : +1;
      }
    }
    else {                                        //SI LE ROBOT TOURNE, METTRE A JOUR L'ANGLE
      //VERIFIER QUE L'ANGLE DU ROBOT RESTE ENTRE 0 ET 3 INCLUS
      robotA = ( robotA + ((motorState == motorStates::RIGHT) ? 1 : -1) + 4) % 4;
    }
}

void think_travel_end() {
  if(robotX == baseX && robotY == baseY) {
    robotState = robotStates::STANDBY;
    pathIndex = 0;
    tickLocal = 0;
    remoteState = remoteStates::NONE;
    IrReceiver.restartTimer();
    return;
  }
  else {
    robotState = robotStates::SERVE;
    pathIndex = 0;
    tickLocal = TIMER_SERVING;
  }
}

void think_travel_blocked() {
  if(robotA%2 == 0) {                       //DECLARER LA CASE DEVANT LE ROBOT COMME ETANT UN MUR
    restaurant[robotY + (robotA == robotAngles::NORTH ? -1 : 1)][robotX] = gridStates::WALL;
  }
  else {
    restaurant[robotY][robotX + (robotA == robotAngles::WEST ? -1 : 1)] = gridStates::WALL;
  }
  Serial.println("too many tries, blocked off obstacle");

  think_travel_start();
  if(robotState == robotStates::STANDBY) {
    Serial.println("The obstacle made the path untraceable. I am going B2B");
    targetX = baseX;
    targetY = baseY;
    think_travel_start();
    if(robotState == robotStates::STANDBY) {
      Serial.println("I can't even do that, I give up completely");
      remoteState = remoteStates::NONE;
      remoteCommandIndex = 0;
      remoteCommand[0] = 0;
      remoteCommand[1] = 0;
      remoteCommand[2] = 0;
      return;
    }
  }
}

void think_travel() {                             //FONCTION QUI GUIDE LE ROBOT DANS L'ETAT "TRAVEL"
  //Serial.println("TRAVEL");
  //CE MOTIF DE CLIGNOTEMENT DESIGNERA QUE LE ROBOT SE DEPLACE
  digitalWrite(LIGHT, (GET_4TH_BIT(tickGlobal)) ? HIGH : LOW );
  
  noInterrupts();
  int motorRightPulseCurrent = motorRightPulse;
  int motorLeftPulseCurrent = motorLeftPulse;
  interrupts();

  //Serial.print(motorRightPulseCurrent);Serial.print("-");Serial.println(motorLeftPulseCurrent);
  //SI NOUS AVONS ATTEINT LE NOMBRE D'IMPULSIONS NECESSAIRES POUR ATTEINDRE NOTRE DESTINATION
  if(((motorRightPulseCurrent + motorLeftPulseCurrent) >> 1) >= motorPulseTarget) {
    if(motorState == motorStates::BACKWARDS) {
      Serial.print("RECOVERED! try n°");Serial.println(retries);
      motorState = motorStates::INERT;
      motor_update();
      if(retries != 3) {
        robotState = robotStates::BLOCKED;
        tickLocal = TIMER_BLOCKED;
        return;
      }
      think_travel_blocked();
      return;
    }
    update_robot_after_action();
    motorState = path[pathIndex++];
    retries = 0;
    motor_update();
    if(motorState == motorStates::INERT) {
      think_travel_end();
    }
    return;
  }

  short motorRightPulseDelta = motorRightPulseCurrent - motorRightPulsePrevious;
  short motorLeftPulseDelta = motorLeftPulseCurrent - motorLeftPulsePrevious;
  //Serial.print(motorRightPulseDelta);Serial.print("-");Serial.println(motorLeftPulseDelta);

  short motorRightSpeedTarget = motorSpeedTarget;
  short motorLeftSpeedTarget = motorSpeedTarget;
  //Serial.print(motorRightSpeedTarget);Serial.print("-");Serial.println(motorLeftSpeedTarget);
  if(motorState == motorStates::FORWARDS || motorState == motorStates::BACKWARDS) {
    short motorHeadingErrorCurrent = motorRightPulseDelta - motorLeftPulseDelta;

    motorHeadingErrorCurrent = motorState == motorStates::BACKWARDS ? -motorHeadingErrorCurrent : motorHeadingErrorCurrent;
    //Serial.println(motorHeadingErrorCurrent);

    float motorHeadingP = MOTOR_KP_HE * motorHeadingErrorCurrent;
    motorHeadingI += MOTOR_KI_HE * motorHeadingErrorCurrent * TIMER_INTERVAL_S;
    motorHeadingI = constrain(motorHeadingI, -0.5f * motorSpeedTarget, 0.5f * motorSpeedTarget);    
    float motorHeadingD = MOTOR_KD_HE * (motorHeadingErrorCurrent - motorHeadingErrorPrevious) / TIMER_INTERVAL_S;
    
    float motorDeltaSplit = motorHeadingP + motorHeadingI + motorHeadingD;
    motorDeltaSplit = constrain(motorDeltaSplit, -0.5f * motorSpeedTarget, 0.5f * motorSpeedTarget);

    motorRightSpeedTarget = (short)max(0.0f, (float)motorRightSpeedTarget - motorDeltaSplit);
    motorLeftSpeedTarget  = (short)max(0.0f,  (float)motorLeftSpeedTarget  + motorDeltaSplit);

    //Serial.print(motorHeadingP);Serial.print(" + ");Serial.print(motorHeadingI);Serial.print(" + ");Serial.println(motorHeadingD);
    motorHeadingErrorPrevious = motorHeadingErrorCurrent;
  }
  //Serial.print(motorRightSpeedTarget);Serial.print("-");Serial.println(motorLeftSpeedTarget);
  short motorRightErrorCurrent = motorRightSpeedTarget - motorRightPulseDelta;
  short motorLeftErrorCurrent = motorLeftSpeedTarget - motorLeftPulseDelta;

  //Serial.print(motorRightErrorCurrent);Serial.print("-");Serial.println(motorLeftErrorCurrent);

  float motorRightP = MOTOR_KP_LR * motorRightErrorCurrent;
  float motorLeftP = MOTOR_KP_LR * motorLeftErrorCurrent;

  motorRightI += MOTOR_KI_LR * motorRightErrorCurrent * TIMER_INTERVAL_S;
  motorLeftI += MOTOR_KI_LR * motorLeftErrorCurrent * TIMER_INTERVAL_S;
  motorRightI = constrain(motorRightI, 0.0f, 255.0f);
  motorLeftI = constrain(motorLeftI, 0.0f, 255.0f);

  float motorRightD = MOTOR_KD_LR * (motorRightErrorCurrent - motorRightErrorPrevious) / TIMER_INTERVAL_S;
  float motorLeftD = MOTOR_KD_LR * (motorLeftErrorCurrent - motorLeftErrorPrevious) / TIMER_INTERVAL_S;

  motorRightPWM = (byte) constrain((int)(motorRightP + motorRightI + motorRightD), MOTOR_MOVEMENT_PWM_MIN, 255);
  motorLeftPWM = (byte) constrain((int)(motorLeftP + motorLeftI + motorLeftD) , MOTOR_MOVEMENT_PWM_MIN, 255);

  analogWrite(MOTOR_ENA, motorRightPWM);
  analogWrite(MOTOR_ENB, motorLeftPWM);

  motorRightErrorPrevious = motorRightErrorCurrent;
  motorLeftErrorPrevious = motorLeftErrorCurrent;
  motorRightPulsePrevious = motorRightPulseCurrent;
  motorLeftPulsePrevious = motorLeftPulseCurrent;

  //Serial.print(motorRightP);Serial.print(" + ");Serial.print(motorRightI);Serial.print(" + ");Serial.println(motorRightD);
  //Serial.print(motorRightPWM);Serial.print("-");Serial.println(motorLeftPWM);

  if(motorState == motorStates::FORWARDS) {
    distance = sonar.ping_cm();
    Serial.println(distance);
    if(
      (distance != 0) && 
      (distance < OBSTACLE_DISTANCE_THRESHOLD)
    ) {                                           //SI LA VALEUR DU CAPTEUR N'EST PAS ERRONNEE EST QU'ELLE N'EST EN DESSOUS DE 5CM
      //robotState = robotStates::BLOCKED;        //DECLAREE LE ROBOT COMME ETANT BLOQUEE
      motorState = motorStates::BACKWARDS;        //FAIRE RECULER LE ROBOT
      retries++;                                  //INCREMENTER LE NOMBRE D'ESSAIS
      int traveled = ((motorRightPulseCurrent + motorLeftPulseCurrent) >> 1);
      motor_update();                             //METTRE A JOUR LES MOTEURS
      //FAIRE RECULER LE ROBOT POUR AUTANT DE TEMPS QU'IL AURAIT AVANCE
      //motorPulseTarget = MOTOR_MOVEMENT_PULSE - traveled;
      motorPulseTarget = traveled;
      Serial.print("BLOCKED! AT (");
      Serial.print(robotX);Serial.print(", ");Serial.print(robotY);Serial.print(", ");Serial.print(robotA);
      Serial.print(") --> ");Serial.print(motorPulseTarget);Serial.print(", ");Serial.println(retries);
      return;
    }
  }
}

void think_serve() {                              //FONCTION QUI GUIDE LE ROBOT DANS L'ETAT "SERVE"
  //CE MOTIF DE CLIGNOTEMENT DESIGNERA QUE LE ROBOT SERT LES CLIENTS
  digitalWrite(LIGHT, (GET_5TH_BIT(tickGlobal) || GET_3RD_BIT(tickGlobal)) ? HIGH : LOW );
  if(tickLocal == 0) {                            //SI LE TEMPS D'ATTENTE S'EST ECOULE
    Serial.print("Enough, go B2B --> ");
    targetX = baseX;                              //REVENIR A LA BASE
    targetY = baseY;
    think_travel_start();
    return;
  }
  tickLocal--;
}

void think_blocked() {                            //FONCTION QUI GUIDE LE ROBOT DANS L'ETAT "BLOCKED"
  //CE MOTIF DE CLIGNOTEMENT DESIGNERA QUE LE ROBOT EST BLOQUE
  digitalWrite(LIGHT, (GET_4TH_BIT(tickGlobal) || GET_2ND_BIT(tickGlobal)) ? HIGH : LOW );
  if(tickLocal == 0) {
    robotState = robotStates::TRAVEL;           //REESAYER NOTRE PRECEDENTE ACTION ET REFAIRE PROMENER LE ROBOT
    motorState = path[pathIndex-1];
    Serial.print("REDO : ACTION N°");Serial.print(pathIndex);
    Serial.print(" (");Serial.print(robotX);Serial.print(", ");Serial.print(robotY);Serial.print(", ");Serial.print(robotA);Serial.println(")");
    motor_update();
    return;
  }
  tickLocal--;
}

void think() {                                    //FONCTION QUI GUIDE LE ROBOT SELON SON ETAT
  switch(robotState) {
    case robotStates::STANDBY :
      think_standby();
      break;
    case robotStates::TRAVEL :
      think_travel();
      break;
    case robotStates::BLOCKED :
      think_blocked();
      break;
    case robotStates::SERVE :
      think_serve();
      break;
  }
}

//*******FONCTIONS_ET_VARIABLES_FIN*******

//***********PREPARATTION_DEBUT***********
void setup() {
  Serial.begin(9600);Serial.println();
  //----------------MOTEUR----------------        //INITIALISER TOUTES LES BROCHES CONCERNANT LES MOTEURS
  pinMode(MOTOR_ENA, OUTPUT); analogWrite(MOTOR_ENA, 0);
  pinMode(MOTOR_IN1, OUTPUT); digitalWrite(MOTOR_IN1, LOW);
  pinMode(MOTOR_IN2, OUTPUT); digitalWrite(MOTOR_IN2, LOW);
  pinMode(MOTOR_ENB, OUTPUT); analogWrite(MOTOR_ENB, 0);
  pinMode(MOTOR_IN3, OUTPUT); digitalWrite(MOTOR_IN3, LOW);
  pinMode(MOTOR_IN4, OUTPUT); digitalWrite(MOTOR_IN4, LOW);
  pinMode(ENCODER_L_C1, INPUT_PULLUP);
  pinMode(ENCODER_R_C1, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_L_C1), 
    interruptMotorEncoderLeft, 
    RISING 
  );
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_R_C1), 
    interruptMotorEncoderRight, 
    RISING 
  );
  motorState = motorStates::INERT;                //INITIALISER L'ETAT DES MOTEURS COMME ETANT IMMOBILE
  motor_update();                                 //ET APPLIQUER LE CHANGEMENT
  //-------------TELECOMMANDE-------------
  remoteState = remoteStates::NONE;               //INITIALISER L'ETAT DE LA TELECOMMANDE COMME ETANT INUTILISE
  IrReceiver.begin(REMOTE, ENABLE_LED_FEEDBACK);  //INITIALISER LA LIBRAIRIE DE LA TELECOMMANDE
  //----------------LIGHT----------------         //INITIALISER LA DEL
  pinMode(LIGHT, OUTPUT); digitalWrite(LIGHT, LOW);
  //----------ULTRASONIC SENSORS----------
  //-------------INTELLIGENCE-------------
  robotState = robotStates::STANDBY;              //INITIALISER L'ETAT DU ROBOT COMME ETANT EN ATTENTE DE NOUVELLES COMMANDES
}
//************PREPARATTION_FIN************

//**************BOUCLE_DEBUT**************
void loop() {
  currentTime = millis();                         //NOUS VOULONS QUE NOTRE ROBOT TRAITE LE PROGRAMME UN QUANTITE PRECISE DE FOIS PAR SECONDE
  //POUR CA, NOUS VERIFIONS QU'UNE QUANTITE DE TEMPS PRECISE S'EST ECOULEE AVANT QUE L'ON TRAITE LE PROGRAMME A NOUVEAU
  if(currentTime - previousTime >= TIMER_INTERVAL_MS) {
    previousTime = currentTime;                   //SI C'EST LE CAS, ON PEUT REINITIALISER LE TEMPS ECOULE ET TRAITER LE PROGARMME
    think();
    tickGlobal++;
  }
}
//***************BOUCLE_FIN***************

