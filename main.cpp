/** Main Test Board
 *
 *   \brief Programme de tests pour le robot NCR 2017
 *   \author H. Angelis
 *   \version alpha_1
 *   \date 15/05/17
 *
 */

#include "mbed.h"
#include "PwmIn.h"
#include "Nucleo_Encoder_16_bits.h"

#define BOUSSOLE_adress     0xC0
#define PIXY_adress         0x54

#define CC_BLOCSIZE         14
#define N_BLOCSIZE          12

#define N_BLOCCODE          0xAA55
#define CC_BLOCCODE         0xAA56


// Comme la nucleo est Little Endian et que l'AS5047D est Big Endian, les codes sont inversés
//  Registre        Valeur (BE) Valeur (LE)
//  NOP             0xC000      0x0003
//  ANGLE           0xFFFF      0xFFFF
//  ERROR           0x4001      0x8002
//  SETTING1 (R)    0xC018      0x1803
//  SETTING2 (R)    0x4019      0x8902
//  SETTING1 (W)    0x0018      0x1800
//  SETTING2 (W)    0x8019      0x9801

#define SPI_READ_NOP        0x0003
#define SPI_READ_ANGLE      0xFFFF
#define SPI_READ_ERROR      0x8002
#define SPI_READ_SETTINGS1  0x1803
#define SPI_READ_SETTINGS2  0x8902
#define SPI_WRTIE_SETTINGS1 0x1800
#define SPI_WRITE_SETTINGS2 0x9801

typedef unsigned char   Byte;
typedef unsigned short  Word;
typedef unsigned long   lWord;

typedef enum {S_monte = 1, S_descente = 0} T_SERVODIR;

typedef enum {none, begin, normal, colorCode, doubleZero} T_pixyState;

typedef union {
    lWord   mot;
    Byte    tab[4];
} T_tmpBuffer;

typedef union {
    Word    mot;
    Byte    tab[2];
} T_structBuffer;

typedef struct {
    Word    checksum;
    Word    signature;
    Word    x;
    Word    y;
    Word    width;
    Word    height;
    Word    angle;
} T_pixyCCBloc;

typedef struct {
    Word    checksum;
    Word    signature;
    Word    x;
    Word    y;
    Word    width;
    Word    height;
} T_pixyNMBloc;

typedef union {
    Byte            tab[14];
    T_pixyCCBloc    CCbloc;
} T_pixyCCData;

typedef union {
    Byte            tab[12];
    T_pixyNMBloc    NMbloc;
} T_pixyNMData;



/** Liste des objets
 *
 *  Serial #4   Pixy
 *  Serial #2   Pc
 *
 *  AnalogIn    C1, C2, C3, LD1, LD2, SD1, SD2, Vbat
 *
 *  DigitalOut  Led1, Led2, Trig1, Trig2, Trig3, En, SensG, SensD
 *
 *  InterruptIn IndexG, IndexD, Echo1, Echo2, Echo3, BP
 *
 *  PwmOut      Pwm_MG, Pwm_MD, Servo
 *
 *  PwmIn       PWMG, PWMD, PWMB
 *
 *  I2C         Bus_I2C
 *
 *  SPI         MotG, MotD
 *
 *  Nucleo_Encoder_16_bits  Gauche, Droite
 *
 *  Ticker      timer
 */

/** Liste des PINs
 *
 * PIN MAP (ordre alphabetique) des PINs de la Nucléo 64 utilisée
 * PA_0    -> Pixy RX (Serial)
 * PA_1    -> Pixy TX (Serial)
 * PA_2    -> PC TX (Serial)
 * PA_3    -> PX RX (Serial)
 * PA_4    -> GP2 SD #2 (Analog In)
 * PA_5    -> LED1 (Digital Out)
 * PA_6    -> CNY3 (Analog In)
 * PA_7    -> CNY2 (Analog In)
 * PA_8    -> Servomoteur (PWM Out)
 * PA_9    -> US Trigger #3 (Digital Out)
 * PA_10   -> US Echo #1 (Pwm In)
 * PA_11   -> US Echo #2 (Pwm In)
 * PA_12   -> SS (SPI Slave Select) (Digital Out)
 * PA_13
 * PA_14
 * PA_15   -> Boussole (Pwm In)
 *
 * PB_0    -> GP2 SD #1 (Analog In)
 * PB_1    -> Position D (Pwm In)
 * PB_2    -> Position G (Pwm In)
 * PB_3    -> PWM Mot D (PWM Out)
 * PB_4    -> Enocdeur Droit A (QE)
 * PB_5    -> Enocdeur Droit B (QE)
 * PB_6    -> Enocdeur Gauche A (QE)
 * PB_7    -> Enocdeur Gauche B (QE)
 * PB_8    -> SCL (I2C)
 * PB_9    -> SDA (I2C)
 * PB_10   -> PWM Mot G (PWM Out)
 * PB_11
 * PB_12   -> US Echo #3 (Pwm In)
 * PB_13   -> SCK Encodeur D (SPI)
 * PB_14   -> MISO Encodeur D (SPI)
 * PB_15   -> MOSI Encodeur D (SPI)
 *
 * PC_0    -> GP2 LD #1 (Analog In)
 * PC_1    -> GP2 LD #2 (Analog In)
 * PC_2    -> US Trigger #2 (Digital Out)
 * PC_3    -> US Trigger #1 (Digital Out)
 * PC_4    -> CNY1 (Analog In)
 * PC_5    -> Vbat (Analog In)
 * PC_6    -> Dir Mot Droit (Digital Out)
 * PC_7    -> I (Encodeur Gauche) (IRQ In)
 * PC_8    -> Dir Mot Gauche (Digital Out)
 * PC_9    -> Enable Moteurs (Digital Out)
 * PC_10   -> SCK Encodeur G (SPI)
 * PC_11   -> MISO Encodeur G (SPI)
 * PC_12   -> MOSI Encodeur G (SPI)
 * PC_13   -> User BP (IRQ In)
 * PC_14
 * PC_15
 *
 * PD_1
 * PD_2    -> Led2 (Digital Out)
 */

Serial      Pixy    (PA_0, PA_1, 230400);
Serial      Pc      (PA_2, PA_3, 9600);

AnalogIn    CNY1    (PC_4);
AnalogIn    CNY2    (PA_7);
AnalogIn    CNY3    (PA_6);
AnalogIn    LD1     (PC_0);
AnalogIn    LD2     (PC_1);
AnalogIn    SD1     (PB_0);
AnalogIn    SD2     (PA_4);
AnalogIn    Vbat    (PC_5);

DigitalOut  Led1    (PA_5);
DigitalOut  Led2    (PD_2);
DigitalOut  Trig1   (PC_3);
DigitalOut  Trig2   (PC_2);
DigitalOut  Trig3   (PA_9);
DigitalOut  En      (PC_9);
DigitalOut  SensG   (PC_8);
DigitalOut  SensD   (PC_6);
DigitalOut  SS      (PA_12);

InterruptIn Echo1   (PA_10);
InterruptIn Echo2   (PA_11);
InterruptIn Echo3   (PB_12);
InterruptIn BP      (PC_13);
InterruptIn IG      (PC_7);
InterruptIn PWMG    (PB_2);
InterruptIn PWMD    (PB_1);

PwmIn       PWMB    (PA_15);

PwmOut      Pwm_MG  (PB_10);
PwmOut      Pwm_MD  (PB_3);
PwmOut      Servo   (PA_8);

I2C         Bus_I2C (PB_9, PB_8);

SPI         SPIG    (PC_12, PC_11, PC_10);
SPI         SPID    (PB_15, PB_14, PB_13);

Nucleo_Encoder_16_bits  Gauche (TIM4);  // A = PB_6, B = PB_7
Nucleo_Encoder_16_bits  Droite (TIM3);  // A = PB_4, B = PB_5

Ticker      tick;
Timer       times;

/** Liste des variables globales
 *
 *  Tick        -> (long)   Compte le nombre de MS écoulé et déclenche l'exécution de la boucle en fonction du temps écoulé.
 *  FlagIG      -> (int)    Indication de la présence de fronts sur l'index de l'encodeur de la roue gauche
 *  FlagID      -> (int)    Indication de la présence de fronts sur l'index de l'encodeur de la roue droite
 *  EchoXStart  -> (long)   Donne le temps en µs de début de l'impulsion d'écho de l'US n°X
 *  DistUSX     -> (float)  Donne la distance en mm mesurée par l'US n°X
 */

// Structure de temps
lWord               Tick = 0, Gperiod, Dperiod;

// Sémaphore d'interruption
int                 FlagUS1 = 0, FlagUS2 = 0, FlagUS3 = 0, FlagPixy = 0, FlagPixyOverflow = 0;
int                 FlagTick = 0, FlagTickLed = 0, BPPressed = 0, nbTurnG = 0, nbTurnD = 0;
int                 Pixy_check = -1;

//  Dialogue avec la Pixy
T_pixyCCData        Pixy_CCFIFO[20];
T_pixyNMData        Pixy_NMFIFO[20];
Byte                Pixy_CCObjet, Pixy_NMObjet;

// Gestion des capteurs Ultrason
long                Echo1Start, Echo2Start, Echo3Start;
double              DistUS1, DistUS2, DistUS3;

/** Liste des interruptions
 *
 */

void tickTime()
{
    Tick++;
    FlagTick = 1;
    if ((Tick%100)==0)  FlagTickLed = 1;
}

void BPevent ()
{
    BPPressed = 1;
}

void PWM_motGRise (void)
{
    static lWord    oldTime;
    lWord           now;
    now = times.read_us();
    Gperiod = now-oldTime;
    oldTime = now;
    nbTurnG++;
}

void PWM_motDRise (void)
{
    static lWord    oldTime;
    lWord           now;
    now = times.read_us();
    Dperiod = now-oldTime;
    oldTime = now;
    nbTurnD++;
}

void echo1Rise ()
{
    Echo1Start = times.read_us();
}

void echo2Rise ()
{
    Echo2Start = times.read_us();
}

void echo3Rise ()
{
    Echo3Start = times.read_us();
}

void echo1Fall ()
{
    DistUS1 = (double)(times.read_us() - Echo1Start)/5.8;
    FlagUS1 = 1;
}

void echo2Fall ()
{
    DistUS2 = (double)(times.read_us() - Echo2Start)/5.8;
    FlagUS2 = 1;
}

void echo3Fall ()
{
    DistUS3 = (double)(times.read_us() - Echo3Start)/5.8;
    FlagUS3 = 1;
}

void getPixyByte ()
{
    static T_tmpBuffer      tmpBuffer;
    static T_structBuffer   msgBuffer;
    static T_pixyState      PIXY_state = none;
    static Byte             byteCount = 0;
    static int              PIXY_synced = 0, dummy;
    int                     i, somme;
    static Byte             PIXY_nbCCObjet = 0, PIXY_wCCObjet = 0;
    static Byte             PIXY_nbNMObjet = 0, PIXY_wNMObjet = 0;

    Pixy_check = 0;

    if (!PIXY_synced) {                                                         // On n'a pas trouvé le START (0x55aa0000)
        tmpBuffer.tab[byteCount] = Pixy.getc();                                 // On stocke l'octet reçu dans la première case dispo du tableau temporaire

        if (byteCount < 3) {                                                    // Si on n'a pas encore reçu les 4 premier octets
            byteCount++;                                                        // On passe à la case suivante du tableau temporaire
        } else {                                                                // Lorsqu'on a 4 octets
            if (tmpBuffer.mot != 0xaa550000) {                                  // Si le code n'est pas le code de START
                for (i=1; i<4; i++) tmpBuffer.tab[i-1] = tmpBuffer.tab[i];      // On décalle les cases du tableau
                byteCount = 3;                                                  // Et on attend le caractère suivant
            } else {                                                            // Si on a trouvé le code de START
                PIXY_synced = 1;                                                // On passe en mode synchronisé
                PIXY_state = begin;
                byteCount = 0;
            }
        }
    }

    if (PIXY_synced) {

        switch (PIXY_state) {
            case begin :                                                        // l'aiguillage est là !

                msgBuffer.tab[byteCount%2] = Pixy.getc();                       // on stocke les octets reçus
                byteCount++;
                if (byteCount == 2) {                                           // Quand on a 2 octets

                    if (msgBuffer.mot == 0xaa55) {                              // Si c'est un bloc normal (code 0xAA55)
                        PIXY_state = normal;                                    // On part vers le traitement spécifique
                    }

                    if (msgBuffer.mot == 0xaa56) {                              // Si c'est un bloc Color Code (code 0xAA56)
                        PIXY_state = colorCode;                                 // On part vers le traitement spécifique
                    }

                    if (msgBuffer.mot == 0) {                                   // Si on a un debut de trame (code 0000)
                        PIXY_state = doubleZero;                                // On part vers le traitement spécifique
                    }
                    if ((PIXY_state == begin) || (PIXY_state == none)) {        // Si c'est autre chose
                        PIXY_synced = 0;                                        // C'est qu'on est perdu donc plus synchronisé.
                        PIXY_state = none;                                      // Ceinture et bretelle
                    }
                    byteCount = 0;
                }
                break;

            case normal :                                                       // Si on a un bloc normal

                Pixy_NMFIFO[PIXY_wNMObjet].tab[byteCount] = Pixy.getc();        // On stocke les octets un à un dans la structure Bloc
                if (byteCount < 11) {                                           // Tant que la structure n'est pas pleine
                    byteCount++;                                                // On passe à l'octet suivant
                } else {                                                        // Quand elle est pleine
                    byteCount = 0;                                              // On réinitialise
                    PIXY_state = begin;                                         // On retourne à l'aiguillage
                    // On calcule la somme de contrôle
                    somme = Pixy_NMFIFO[PIXY_wNMObjet].NMbloc.signature + Pixy_NMFIFO[PIXY_wNMObjet].NMbloc.x + Pixy_NMFIFO[PIXY_wNMObjet].NMbloc.y + Pixy_NMFIFO[PIXY_wNMObjet].NMbloc.width + Pixy_NMFIFO[PIXY_wNMObjet].NMbloc.height;

                    if (somme == Pixy_NMFIFO[PIXY_wNMObjet].NMbloc.checksum) {  // Si le checksum est bon, on valide la réception
                        if (PIXY_wNMObjet < 19)     PIXY_wNMObjet++;            // On incrémente le pointeur d'écriture dans la FIFO Objet
                        else                        PIXY_wNMObjet = 0;
                        if (PIXY_nbNMObjet < 19)    PIXY_nbNMObjet++;           // On dit que l'on a un objet CC de plus
                        else                        FlagPixyOverflow = 1;       // Si on a plus de 20 CC objets (en attente) => Overflow
                    }
                }
                break;

            case colorCode :                                                    // Si on a un bloc colorCode

                Pixy_CCFIFO[PIXY_wCCObjet].tab[byteCount] = dummy;              // On stocke les octets un à un dans la structure CCBloc
                if (byteCount < 13) byteCount++;                                // tant que la structure n'est pas pleine on passe à l'octet suivant
                else {                                                          // Quand elle est pleine
                    byteCount = 0;                                              // On réinitialise
                    PIXY_state = begin;                                         // On retourne à l'aiguillage
                    // On calcule la somme de contrôle
                    somme = Pixy_CCFIFO[PIXY_wCCObjet].CCbloc.signature + Pixy_CCFIFO[PIXY_wCCObjet].CCbloc.x + Pixy_CCFIFO[PIXY_wCCObjet].CCbloc.y + Pixy_CCFIFO[PIXY_wCCObjet].CCbloc.width + Pixy_CCFIFO[PIXY_wCCObjet].CCbloc.height + Pixy_CCFIFO[PIXY_wCCObjet].CCbloc.angle;

                    if (somme == Pixy_CCFIFO[PIXY_wCCObjet].CCbloc.checksum) {  // Si le checksum est bon
                        if (PIXY_wCCObjet < 19)     PIXY_wCCObjet++;            // On incrémente le pointeur d'écriture dans la FIFO CCObjet
                        else                        PIXY_wCCObjet = 0;
                        if (PIXY_nbCCObjet < 19)    PIXY_nbCCObjet++;           // On dit que l'on a un objet CC de plus à traiter
                        else                        FlagPixyOverflow = 1;       // Si on a plus de 20 CC objets (en attente) => Overflow
                    }
                }
                break;

            case doubleZero :                                                   // Si on a reçu le code de début d'une nouvelle trame.

                msgBuffer.tab[byteCount%2] = Pixy.getc();                       // on stocke les octets reçus
                byteCount++;
                if (byteCount == 2) {                                           // Quand on a 2 octets
                    if (msgBuffer.mot == 0xaa55) {                              // On doit impérativement trouver le code 0xAA55
                        PIXY_state = begin;                                     // Si c'est le cas, alors tout va bien et on va à l'aiguillage
                        Pixy_NMObjet = PIXY_nbNMObjet;                          // On met à jour les variables pour le traitement
                        Pixy_CCObjet = PIXY_nbCCObjet;
                        PIXY_nbCCObjet = 0;
                        PIXY_nbNMObjet = 0;
                        FlagPixy = 1;                                           // On valide le traitement de la trame précédente.
                    } else {                                                    // Si on trouve autre chose
                        PIXY_synced = 0;                                        // C'est qu'on est perdu donc plus synchronisé.
                        PIXY_state = none;                                      // Ceinture et bretelle
                    }
                    byteCount = 0;
                }
                break;
        }
    }
}


int main()
{

    int         I2C_check = -1, BOUSSOLE_check = -1 /*, SPI2_check = -1, SPI3_check = -1, MOTG_check = -1, MOTD_check = -1*/;

    Byte        PIXY_rCCObjet = 0, PIXY_rNMObjet = 0;
    int         PIXY_objet;

    int         SERVO_pulseWidth = 400, SERVO_max = 1400, SERVO_min = 400;
    T_SERVODIR  SERVO_dir = S_monte;

    char        MENU_choix = 0;

    char        BOUSSOLE_status[1] = {0};
    char        I2C_registerValue[4];
    double      BOUSSOLE_periode;

    double      CAP_I2C, CAP_PWM;
    double      SD1_val, SD2_val, LD1_val, LD2_val, CNY1_val, CNY2_val, CNY3_val, Vbat_val;
    double      SD1_dist, SD2_dist, LD1_dist, LD2_dist;

    int         MOTG_evol = 1, MOTD_evol = 1;
    double      MOTG_duty = 0.5, MOTD_duty = 0.5;

    times.reset();
    times.start();

    // Initialisation des interruptions
    tick.attach(&tickTime, 0.001);

    BP.rise     (&BPevent);

    Echo1.rise  (&echo1Rise);
    Echo2.rise  (&echo2Rise);
    Echo3.rise  (&echo3Rise);
    Echo1.fall  (&echo1Fall);
    Echo2.fall  (&echo2Fall);
    Echo3.fall  (&echo3Fall);

    PWMG.rise   (&PWM_motGRise);
    PWMD.rise   (&PWM_motDRise);

    Pixy.attach (&getPixyByte);

    BP.enable_irq();
    IG.enable_irq();
    Echo1.enable_irq();
    Echo2.enable_irq();
    Echo3.enable_irq();
    PWMG.enable_irq();
    PWMD.enable_irq();

    // Initialisation des périphériques
    // Bus I2C
    Bus_I2C.frequency (100000);

    // PWM des moteurs
    Pwm_MG.period_us(50);
    Pwm_MD.period_us(50);
    En = 0;

    // Bus SPI
    SPIG.format (16,1);
    SPIG.frequency (1000000);

    SPID.format (16,1);
    SPID.frequency (1000000);

    SS = 1;

    // Led
    Led2 = 0;

    Servo.period_ms (20);
    Servo.pulsewidth_us(200);

    while(1) {

        do {
            Led1 = 0;
            Pc.printf ("\n\n\n\n\rProgramme de test\n\n\rEntrez le code du test a effectuer :\n\n");
            Pc.printf ("\r1- Capteurs Ultra Son (les 3)\n");
            Pc.printf ("\r2- Boussole et I2C\n");
            Pc.printf ("\r3- Capteurs GP2 (les 4)\n");
            Pc.printf ("\r4- Capteurs CNY70 (les 3)\n");
            Pc.printf ("\r5- VBAT \t! erreur de composant la mesure est fausse !\n");
            Pc.printf ("\r6- Moteur Gauche\n");
            Pc.printf ("\r7- Moteur Droit\n");
            Pc.printf ("\r8- Servomoteur\n");
            Pc.printf ("\r9- PIXY (CMUCAM5)\n");
            MENU_choix = Pc.getc ();
        } while (((MENU_choix-'0')<1) || ((MENU_choix-'0')>9));

        switch (MENU_choix-'0') {

            case 1 :
                Pc.printf ("\n\n\rTest des captreurs Ultrason\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");
                do {

                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;
                    }

                    // Gestion des US
                    if (((Tick%150)==0) && FlagTick) {
                        Trig1 = 1;
                        wait_us(20);
                        Trig1 = 0;
                        FlagTick = 0;
                    }

                    if (((Tick%150)==50) && FlagTick) {
                        Trig2 = 1;
                        wait_us(20);
                        Trig2 = 0;
                        FlagTick = 0;
                    }

                    if (((Tick%150)==100) && FlagTick) {
                        Trig3 = 1;
                        wait_us(20);
                        Trig3 = 0;
                        FlagTick = 0;
                    }

                    if (FlagUS1==1) {
                        Pc.printf ("\rUS 1 = %04d mm", (int)DistUS1);
                        FlagUS1 = 0;
                    }

                    if (FlagUS2==1) {
                        Pc.printf ("\r\t\t\tUS 2 = %04d mm", (int)DistUS2);
                        FlagUS2 = 0;
                    }

                    if (FlagUS3==1) {
                        Pc.printf ("\r\t\t\t\t\t\tUS 3 = %04d mm", (int)DistUS3);
                        FlagUS3 = 0;
                    }
                } while(!Pc.readable());
                MENU_choix = Pc.getc();
                break;

            case 2 :
                Pc.printf ("\n\n\rBoussole\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");

                Pc.printf ("\n\rVerif du bus I2C :");
                I2C_check = Bus_I2C.write (BOUSSOLE_adress,BOUSSOLE_status,1,false);
                if (I2C_check==0) {
                    Pc.printf (" OK\n");
                    Bus_I2C.write(BOUSSOLE_adress,BOUSSOLE_status, 1, true);
                    Bus_I2C.read (BOUSSOLE_adress,I2C_registerValue,4);
                    Pc.printf ("\rVersion Firmware boussole : %03d\n", I2C_registerValue[0]);
                } else {
                    Pc.printf (" FAIL\n");
                }

                BOUSSOLE_periode = PWMB.period();
                Pc.printf ("\rVerif de la PWM :");
                if ((BOUSSOLE_periode > 0.11) || (BOUSSOLE_periode < 0.06)) {
                    Pc.printf (" FAIL\n\n");
                } else {
                    Pc.printf (" OK\n\n");
                    BOUSSOLE_check = 0;
                }

                do {
                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;
                    }

                    if (((Tick%150)==0) && FlagTick) {
                        FlagTick = 0;
                        if (BOUSSOLE_check==0) {
                            CAP_PWM = ((PWMB.pulsewidth()*1000)-1)*10;
                            Pc.printf ("\r PWM = %4.1lf", CAP_PWM);
                        }
                        if (I2C_check==0) {
                            Bus_I2C.write(BOUSSOLE_adress,BOUSSOLE_status, 1, true);
                            Bus_I2C.read (BOUSSOLE_adress,I2C_registerValue,4);
                            CAP_I2C = (double)(((unsigned short)I2C_registerValue[2]<<8)+(unsigned short)I2C_registerValue[3])/10.0;
                            Pc.printf ("\r\t\t I2C = %4.1lf", CAP_I2C);
                        }
                    }
                } while(!Pc.readable());
                MENU_choix = Pc.getc();
                break;

            case 3 :
                Pc.printf ("\n\n\rGP2xx\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");
                do {
                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;

                        SD1_val = SD1;
                        SD2_val = SD2;
                        LD1_val = LD1;
                        LD2_val = LD2;

                        if (SD1_val < 0.06) {
                            SD1_val = 0;
                            SD1_dist = 40;
                        } else {
                            SD1_dist = 11.611/(SD1_val*3.3-0.0237);
                            if (SD1_dist > 40) SD1_dist = 40;
                        }

                        if (SD2_val < 0.06) {
                            SD2_val = 0;
                            SD2_dist = 40;
                        } else {
                            SD2_dist = 11.611/(SD2_val*3.3-0.0237);
                            if (SD2_dist > 40) SD2_dist = 40;
                        }

                        if (LD1_val < 0.1) {
                            LD1_val = 0;
                            LD1_dist = 150;
                        } else {
                            LD1_dist = 59.175/(LD1_val*3.3-0.0275);
                            if (LD1_dist > 150) LD1_dist = 150;
                        }

                        if (LD2_val < 0.1) {
                            LD2_val = 0;
                            LD2_dist = 150;
                        } else {
                            LD2_dist = 59.175/(LD2_val*3.3-0.0275);
                            if (LD2_dist > 150) LD2_dist = 150;
                        }

                        Pc.printf ("\r SD1 = %3.1lf cm - SD2 = %3.1lf cm - LD1 = %4.1lf cm - LD2 = %4.1lf cm", SD1_dist, SD2_dist, LD1_dist, LD2_dist);
                    }
                } while(!Pc.readable());
                MENU_choix = Pc.getc();
                break;

            case 4 :
                Pc.printf ("\n\n\rCNY70\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");
                do {
                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;

                        CNY1_val = CNY1;
                        CNY2_val = CNY2;
                        CNY3_val = CNY3;

                        Pc.printf ("\r CNY1 = %3.2lf V\t CNY2 = %3.2lf V\t CNY3 = %3.2lf V", CNY1_val*3.3, CNY2_val*3.3, CNY3_val*3.3);
                    }
                } while(!Pc.readable());
                MENU_choix = Pc.getc();
                break;

            case 5 :
                Pc.printf ("\n\n\rVbat\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");
                do {
                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;

                        Vbat_val = Vbat;
                        Pc.printf ("\rVbat = %5.3lf V", Vbat_val*3.3*4.3);
                    }
                } while(!Pc.readable());
                MENU_choix = Pc.getc();
                break;

            case 6 :
                Pc.printf ("\n\n\rMoteur Gauche\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");

                En = 1;
                SensG = 1;
                Led2 = 1;
                MOTG_duty = 0.5;
                MOTG_evol = 1;
                Pwm_MG = 0.5;

                do {
                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;
                    }
                    if (BPPressed) {
                        BPPressed = 0;
                        SensG = !SensG;
                        Led2 = !Led2;
                    }

                    if (((Tick%1000)==0) && FlagTick) {
                        FlagTick = 0;
                        if (MOTG_evol) {
                            if (MOTG_duty < 0.9) MOTG_duty += 0.1;
                            else {
                                MOTG_evol = 0;
                                MOTG_duty = 0.9;
                            }
                        } else {
                            if (MOTG_duty > 0.1) MOTG_duty -= 0.1;
                            else {
                                MOTG_evol = 1;
                                MOTG_duty = 0.1;
                            }
                        }
                    }

                    Pwm_MG = MOTG_duty;
                    Pc.printf ("\rPWM = %2.1lf => Pos = %10ld Tick, Period = %ldus", MOTG_duty, Gauche.GetCounter(), Gperiod);


                } while(!Pc.readable());
                MENU_choix = Pc.getc();

                En = 0;
                if (SensG == 0) Pwm_MG = 0;
                else            Pwm_MG = 1;

                break;

            case 7 :
                Pc.printf ("\n\n\rMoteur Droit\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");

                En = 1;
                SensD = 1;
                Led2 = 1;
                MOTD_duty = 0.5;
                MOTD_evol = 1;
                Pwm_MD = 0.5;

                do {
                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;
                    }
                    if (BPPressed) {
                        BPPressed = 0;
                        SensD = !SensD;
                        Led2 = !Led2;
                    }

                    if (((Tick%1000)==0) && FlagTick) {
                        FlagTick = 0;
                        if (MOTD_evol) {
                            if (MOTD_duty < 0.9) MOTD_duty += 0.1;
                            else {
                                MOTD_evol = 0;
                                MOTD_duty = 0.9;
                            }
                        } else {
                            if (MOTD_duty > 0.1) MOTD_duty -= 0.1;
                            else {
                                MOTD_evol = 1;
                                MOTD_duty = 0.1;
                            }
                        }
                    }
                    Pwm_MD = MOTD_duty;
                    Pc.printf ("\rPWM = %2.1lf => Pos = %10ld Tick, Period = %ldus", MOTD_duty, Droite.GetCounter(), Dperiod);

                } while(!Pc.readable());
                MENU_choix = Pc.getc();

                En = 0;
                if (SensD == 0) Pwm_MD = 0;
                else            Pwm_MD = 1;

                break;

            case 8 :
                Pc.printf ("\n\n\rServo Moteur\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");

                do {
                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;
                    }

                    if (((Tick%250)==0) && FlagTick) {
                        FlagTick = 0;
                        if (SERVO_dir == S_monte) {
                            if (SERVO_pulseWidth < (SERVO_max - 100))   SERVO_pulseWidth +=100;
                            else {
                                SERVO_dir = S_descente;
                                SERVO_pulseWidth = SERVO_max;
                            }
                        } else {
                            if (SERVO_pulseWidth > (SERVO_min + 100))   SERVO_pulseWidth -=100;
                            else {
                                SERVO_dir = S_monte;
                                SERVO_pulseWidth = SERVO_min;
                            }
                        }

                        Servo.pulsewidth_us (SERVO_pulseWidth);
                        Pc.printf ("\rPulse = %d",SERVO_pulseWidth);
                    }

                } while(!Pc.readable());
                MENU_choix = Pc.getc();
                break;

            case 9 :
                Pc.printf ("\n\n\rPixy\n");
                Pc.printf ("\rAppuyez sur Entree pour quitter\n");

                if (Pixy_check == 0)    Pc.printf ("\n\rPIXY is ALIVE\n");
                else                    Pc.printf ("\n\rPIXY don't talk\n");

                do {
                    if (FlagPixy) {

                        if (FlagPixyOverflow) {
                            Pc.printf ("\rSome Data were lost\n");
                            FlagPixyOverflow = 0;
                        }
                        for (PIXY_objet = 0; PIXY_objet < Pixy_NMObjet; PIXY_objet++) {
                            Pc.printf ("\rNMobj #%hd/%hd : sig = %hd : X=%5hd, Y=%5hd (W=%5hd, H=%5hd)\n", PIXY_objet+1, Pixy_NMObjet, Pixy_NMFIFO[PIXY_rNMObjet].NMbloc.signature, Pixy_NMFIFO[PIXY_rNMObjet].NMbloc.x, Pixy_NMFIFO[PIXY_rNMObjet].NMbloc.y, Pixy_NMFIFO[PIXY_rNMObjet].NMbloc.width, Pixy_NMFIFO[PIXY_rNMObjet].NMbloc.height);

                            if (PIXY_rNMObjet<19)   PIXY_rNMObjet++;
                            else                    PIXY_rNMObjet = 0;
                        }
                        Pixy_NMObjet = 0;

                        for (PIXY_objet = 0; PIXY_objet < Pixy_CCObjet; PIXY_objet++) {
                            Pc.printf ("\rCCobj #%hd/%hd : sig = %hd : X=%5hd, Y=%5hd (W=%5hd, H=%5hd)\n", PIXY_objet+1, Pixy_CCObjet, Pixy_CCFIFO[PIXY_rCCObjet].CCbloc.signature, Pixy_CCFIFO[PIXY_rCCObjet].CCbloc.x, Pixy_CCFIFO[PIXY_rCCObjet].CCbloc.y, Pixy_CCFIFO[PIXY_rCCObjet].CCbloc.width, Pixy_CCFIFO[PIXY_rCCObjet].CCbloc.height);

                            if (PIXY_rCCObjet<19)   PIXY_rCCObjet++;
                            else                    PIXY_rCCObjet = 0;
                        }
                        Pixy_CCObjet = 0;
                        Pc.printf("\n\r");
                        FlagPixy = 0;
                    }

                    if (FlagTickLed) {
                        Led1 = !Led1;
                        FlagTickLed = 0;
                    }
                } while(!Pc.readable());
                MENU_choix = Pc.getc();
                break;

        }
    }
}
