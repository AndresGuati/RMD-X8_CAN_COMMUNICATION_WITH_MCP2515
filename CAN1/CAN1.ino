//Declaracion de librerias
#include <SPI.h>          //Libreria para SPI
#include <mcp2515.h>      //Libraria para CAN

//Declaracion de botones - prueba manual
#define UP    4
#define DOWN  3
#define UP2   7
#define DOWN2 8
#define UP3   6
#define DOWN3 5

#include <Separador.h>

Separador s;

//Delcaracion velocidad de prueba motores
#define StepValue 500 //500 o 200 o 3000

//Declaracion pin de mcp2515
MCP2515 mcp2515(10); // chip select pin 10

//Declaracion trama de datos CAN motores 1-7
struct can_frame canMsg1; //Motor1
struct can_frame canMsg2; //Motor2
struct can_frame canMsg3; //Motor3
struct can_frame canMsg4; //Motor3
struct can_frame canMsg5; //Motor3
struct can_frame canMsg6; //Motor3
struct can_frame canMsg7; //Motor3

//Declaracion posiciones en bits motores 1-7
long GenPos=0;
long GenPos2=0;
long GenPos3=0;
long GenPos4=0;
long GenPos5=0;
long GenPos6=0;
long GenPos7=0;

//Declaracion posiciones en grados motores 1-7
long GenPosD=0;
long GenPos2D=0;
long GenPos3D=0;
long GenPos4D=0;
long GenPos5D=0;
long GenPos6D=0;
long GenPos7D=0;


int selbrazo;
int CompReady;
int rutina;
int manual;
int articulacion;
int pos_inicial;
int pos_final;
int vel;
int fuerza;
int pausa;
int PEmergencia;

String inputString = "";
bool stringComplete = false;

long GenVel=800; //800 o 500
int dir=0;
int cont=0;

long GenTor=0;

unsigned int buf[8];

int pinAnalogo = A0;
int pinAnalogo1 = A1;
int valorAnalogo;
String dato;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
long Pos1D=0;long Tor1D=0; long Vel1D=0;
long Pos1CC=0;
long Pos1C=0;long Tor1C=0; long Vel1C=0;
long Pos1_1C=0;
long Acum1=0;
long t1=0;
long t1_1=0;
long m1=0;
long Cont1=0;long cont2=0;long cont3=0;long cont4=0;long cont5=0;long cont6=0;long cont7=0;

int wait1=0;
boolean flag1=false;
int Kp1;int Kd1;
long DataM1[9];
long DataM2[9];
long DataM3[9];
long DataM4[9];
long DataM5[9];
long DataM6[9];
long DataM7[9];

byte SendData[30];

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

//Declaracion variable de envio ID
int IDM1; int IDM2; int IDM3; int IDM4; int IDM5; int IDM6; int IDM7;

void setup() {  

  //Inicilizacion serial y SPI  
  Serial.begin(115200);
  SPI.begin();

  //Inicializacion modulo mcp2515 y configuracion
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS,MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();

  //Definicion del ID de los motores 
  canMsg1.can_id  = 0x141;canMsg1.can_dlc = 8;               
  canMsg2.can_id  = 0x142;canMsg2.can_dlc = 8;               
  canMsg3.can_id  = 0x143;canMsg3.can_dlc = 8;               
  canMsg4.can_id  = 0x144;canMsg4.can_dlc = 8;               
  canMsg5.can_id  = 0x145;canMsg5.can_dlc = 8;               
  canMsg6.can_id  = 0x146;canMsg6.can_dlc = 8;               
  canMsg7.can_id  = 0x147;canMsg7.can_dlc = 8;               

  
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
  //Lectura posiciones iniciales
  //Envio datos CAN Motor 1
  canMsg1.data[0] = 0x94;canMsg1.data[1,2,3,4,5,6,7] = 0x00;mcp2515.sendMessage(&canMsg1);
   
  //Recepcion datos CAN Motor 1
  unsigned int EstPos = (canMsg1.data[7] << 8) | canMsg1.data[6];
  if (EstPos>18000){GenPos = -36000 + EstPos;
    }else{GenPos = EstPos;}
  delay(1);
  
  //Lectura posiciones iniciales
  //Envio datos CAN Motor 2
  canMsg2.data[0] = 0x94;canMsg2.data[1,2,3,4,5,6,7] = 0x00;mcp2515.sendMessage(&canMsg2);     //sends the CAN message
  
  //Recepcion datos CAN Motor 2
  unsigned int EstPos2 = (canMsg2.data[7] << 8) | canMsg2.data[6];
  if (EstPos2>18000){GenPos2 = -36000 + EstPos2;
    }else{GenPos2 = EstPos2;}
  delay(1);

  //Lectura posiciones iniciales
  //Envio datos CAN Motor 3
  canMsg3.data[0] = 0x94;canMsg3.data[1,2,3,4,5,6,7] = 0x00;mcp2515.sendMessage(&canMsg3);     
  
  //Recepcion datos CAN Motor 3
  unsigned int EstPos3 = (canMsg3.data[7] << 8) | canMsg3.data[6];
  if (EstPos3>18000){GenPos3 = -36000 + EstPos3;
    }else{GenPos3 = EstPos3;}
  delay(1);

  //Lectura posiciones iniciales
  //Envio datos CAN Motor 4
  canMsg4.data[0] = 0x94;canMsg4.data[1,2,3,4,5,6,7] = 0x00;mcp2515.sendMessage(&canMsg4);     
  
  //Recepcion datos CAN Motor 4
  unsigned int EstPos4 = (canMsg4.data[7] << 8) | canMsg4.data[6];
  if (EstPos4>18000){GenPos4= -36000 + EstPos4;
    }else{GenPos4 = EstPos4;}
  delay(1);

  //Lectura posiciones iniciales
  //Envio datos CAN Motor 5
  canMsg5.data[0] = 0x94;canMsg5.data[1,2,3,4,5,6,7] = 0x00;mcp2515.sendMessage(&canMsg5);     
  
  //Recepcion datos CAN Motor 5
  unsigned int EstPos5 = (canMsg5.data[7] << 8) | canMsg5.data[6];
  if (EstPos5>18000){GenPos5= -36000 + EstPos5;
    }else{GenPos5 = EstPos5;}
  delay(1);

  //Lectura posiciones iniciales
  //Envio datos CAN Motor 6
  canMsg6.data[0] = 0x94;canMsg6.data[1,2,3,4,5,6,7] = 0x00;mcp2515.sendMessage(&canMsg6);     
  
  //Recepcion datos CAN Motor 6
  unsigned int EstPos6 = (canMsg6.data[7] << 8) | canMsg6.data[6];
  if (EstPos6>18000){GenPos6= -36000 + EstPos6;
    }else{GenPos6 = EstPos6;}
  delay(1);

  //Lectura posiciones iniciales
  //Envio datos CAN Motor 7
  canMsg7.data[0] = 0x94;canMsg7.data[1,2,3,4,5,6,7] = 0x00;mcp2515.sendMessage(&canMsg7);     
  
  //Recepcion datos CAN Motor 7
  unsigned int EstPos7 = (canMsg7.data[7] << 8) | canMsg7.data[6];
  if (EstPos7>18000){GenPos7= -36000 + EstPos7;
    }else{GenPos7 = EstPos7;}
  delay(1);

  Serial.print(GenPos);Serial.print(" , ");Serial.print(GenPos2);Serial.print(" , ");Serial.print(GenPos3);Serial.print(" , ");
  Serial.print(GenPos4);Serial.print(" , ");Serial.print(GenPos5);Serial.print(" , ");Serial.print(GenPos6);
  Serial.print(" , ");Serial.print(GenPos7);

  delay(100);

  

//  //Escribir valores PID motores
//  //Envio datos CAN Motor 1
//  canMsg1.data[0] = 0x31;
//  canMsg1.data[1] = 0x00;
//  canMsg1.data[2] = 5;
//  canMsg1.data[3] = 10;
//  canMsg1.data[4] = 15;
//  canMsg1.data[5] = 6;
//  canMsg1.data[6] = 15;
//  canMsg1.data[7] = 6;
//  mcp2515.sendMessage(&canMsg1);   
//  delay(1);

  delay(1000);  
}


void loop() {

  if(stringComplete){
    inputString = "";
    stringComplete= false;
  }   
  
  SendData[0]='I';
  
  //Envio de comandos
  SendData[1]=DataM1[1];
  SendData[2]=DataM2[1];
  SendData[3]=DataM3[1];
  SendData[4]=DataM3[1];
  SendData[5]=DataM3[1];
  SendData[6]=DataM3[1];
  SendData[7]=DataM3[1];

  //ENvio de torques
  SendData[8]=DataM1[4];
  SendData[9]=DataM2[4];
  SendData[10]=DataM3[4];
  SendData[11]=DataM3[4];
  SendData[12]=DataM3[4];
  SendData[13]=DataM3[4];
  SendData[14]=DataM3[4];
  
  //Envio de velocidades
  SendData[15]=DataM1[6];
  SendData[16]=DataM2[6];
  SendData[17]=DataM3[6];
  SendData[18]=DataM3[6];
  SendData[19]=DataM3[6];
  SendData[20]=DataM3[6];
  SendData[21]=DataM3[6];

  //Envio de posiciones
  SendData[22]=DataM1[8];
  SendData[23]=DataM2[8];
  SendData[24]=DataM3[8];
  SendData[25]=DataM3[8];
  SendData[26]=DataM3[8];
  SendData[27]=DataM3[8];
  SendData[28]=DataM3[8];

  SendData[29]='F';
  //SendData[30]='\n';
  Serial.write(SendData,30);
  
  byte inf[10]={1,2,3,4,5,6,7,2,3,5};
  //Serial.write(inf,10);

  delay(1);

    GenPos2=15000;
    GenPos3=-220000;
    GenPos4=60000;
    //GenPos6=-10000;
    GenPos7=-11000;
    GenPos5=50000;

    cont6+=1;
    if(cont6<500){GenPos6=1000;}
    else if(cont6>=400 && cont6<=800){GenPos6=-22000;}
    else{cont6=0;}
    Serial.println(GenPos6);
    
    

//    cont3+=1;
//    if(cont3<500){GenPos3=GenPos3+100;}
//    else if(cont3>=500 && cont3<=2000){GenPos3=GenPos3-100;}
//    else{cont3=0;}
//    Serial.println(GenPos3);
  

  //Envio datos CAN motor 1
  canMsg1.data[0] = 0xA4;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = GenVel;
  canMsg1.data[3] = GenVel >> 8;
  canMsg1.data[4] = GenPos;
  canMsg1.data[5] = GenPos >> 8;
  canMsg1.data[6] = GenPos >> 16;
  canMsg1.data[7] = GenPos >> 24;
  mcp2515.sendMessage(&canMsg1);   
  if (canMsg1.can_id=0x141){
     CANReadData(canMsg1,canMsg1.can_id,canMsg1.can_dlc);
  }
    
  //Envio datos CAN motor 2
  canMsg2.data[0] = 0xA4;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = GenVel;
  canMsg2.data[3] = GenVel >> 8;
  canMsg2.data[4] = GenPos2;
  canMsg2.data[5] = GenPos2 >> 8;
  canMsg2.data[6] = GenPos2 >> 16;
  canMsg2.data[7] = GenPos2 >> 24;
  mcp2515.sendMessage(&canMsg2); 
  if (canMsg2.can_id=0x142){
    CANReadData(canMsg2,canMsg2.can_id,canMsg2.can_dlc);
  }
  
  //Envio datos CAN motor 3
  canMsg3.data[0] = 0xA4;
  canMsg3.data[1] = 0x00;
  canMsg3.data[2] = GenVel;
  canMsg3.data[3] = GenVel >> 8;
  canMsg3.data[4] = GenPos3;
  canMsg3.data[5] = GenPos3 >> 8;
  canMsg3.data[6] = GenPos3 >> 16;
  canMsg3.data[7] = GenPos3 >> 24;
  mcp2515.sendMessage(&canMsg3); 
  if (canMsg3.can_id=0x143){
    CANReadData(canMsg3,canMsg3.can_id,canMsg3.can_dlc);
  }
  
  //Envio datos CAN motor 4
  canMsg4.data[0] = 0xA4;
  canMsg4.data[1] = 0x00;
  canMsg4.data[2] = GenVel;
  canMsg4.data[3] = GenVel >> 8;
  canMsg4.data[4] = GenPos4;
  canMsg4.data[5] = GenPos4 >> 8;
  canMsg4.data[6] = GenPos4 >> 16;
  canMsg4.data[7] = GenPos4 >> 24;
  mcp2515.sendMessage(&canMsg4); 
  if (canMsg4.can_id=0x144){
    CANReadData(canMsg4,canMsg4.can_id,canMsg4.can_dlc);
  }
  
  //Envio datos CAN motor 5
  canMsg5.data[0] = 0xA4;
  canMsg5.data[1] = 0x00;
  canMsg5.data[2] = GenVel;
  canMsg5.data[3] = GenVel >> 8;
  canMsg5.data[4] = GenPos5;
  canMsg5.data[5] = GenPos5 >> 8;
  canMsg5.data[6] = GenPos5 >> 16;
  canMsg5.data[7] = GenPos5 >> 24;
  mcp2515.sendMessage(&canMsg5); 
  if (canMsg5.can_id=0x145){
    CANReadData(canMsg5,canMsg5.can_id,canMsg5.can_dlc);
  }
  
  //Envio datos CAN motor 6
  canMsg6.data[0] = 0xA4;
  canMsg6.data[1] = 0x00;
  canMsg6.data[2] = 100;
  canMsg6.data[3] = 100 >> 8;
  canMsg6.data[4] = GenPos6;
  canMsg6.data[5] = GenPos6 >> 8;
  canMsg6.data[6] = GenPos6 >> 16;
  canMsg6.data[7] = GenPos6 >> 24;
  mcp2515.sendMessage(&canMsg6); 
  if (canMsg6.can_id=0x146){
    CANReadData(canMsg6,canMsg6.can_id,canMsg6.can_dlc);
  }
  
  //Envio datos CAN motor 7
  canMsg7.data[0] = 0xA4;
  canMsg7.data[1] = 0x00;
  canMsg7.data[2] = 100;
  canMsg7.data[3] = 100 >> 8;
  canMsg7.data[4] = GenPos7;
  canMsg7.data[5] = GenPos7 >> 8;
  canMsg7.data[6] = GenPos7 >> 16;
  canMsg7.data[7] = GenPos7 >> 24;
  mcp2515.sendMessage(&canMsg7); 
  if (canMsg7.can_id=0x147){
    CANReadData(canMsg7,canMsg7.can_id,canMsg7.can_dlc);
  }

  delay(1);
  t1=millis();
  
}

void fromInttoBytes(byte* bytes,long lng){
  bytes[0] = (byte) ((lng & 0xFF00) >> 8);
  bytes[1] = (byte) ((lng & 0x00FF)     );
}


void serialEvent(){
  while(Serial.available()){
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n'){
      stringComplete = true;
      selbrazo = (s.separa(inputString,',',0)).toInt();
      CompReady = (s.separa(inputString,',',1)).toInt();
      rutina = (s.separa(inputString,',',2)).toInt();
      manual = (s.separa(inputString,',',3)).toInt();
      articulacion = (s.separa(inputString,',',4)).toInt();
      pos_inicial = (s.separa(inputString,',',5)).toInt();
      pos_final = (s.separa(inputString,',',6)).toInt();
      vel = (s.separa(inputString,',',7)).toInt();
      fuerza = (s.separa(inputString,',',8)).toInt();
      pausa = (s.separa(inputString,',',9)).toInt();
      PEmergencia = (s.separa(inputString,',',10)).toInt();
    }
    delay(1);
  }
}


void CANReadData(can_frame canMsg, int can_id, int can_dlc){
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){
    if (can_id==0x141){
      DataM1[0]=can_id; 
      for (int i = 1; i<=can_dlc; i++){
        DataM1[i]=canMsg.data[i-1];}
    }else if (can_id==0x142){
      DataM2[0]=can_id; 
      for (int i = 1; i<=can_dlc; i++){
        DataM2[i]=canMsg.data[i-1];}
    }else if (can_id==0x143){
      DataM3[0]=can_id; 
      for (int i = 1; i<=can_dlc; i++){
        DataM3[i]=canMsg.data[i-1];}
    }
  }
  delay(1);
}