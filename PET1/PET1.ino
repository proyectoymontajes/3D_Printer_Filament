

int ciclo1 = 1000;

long tiempo1;
long tiempoI;
long periodI = 1000;
long tiempo;
int ONtime = 3;
int OFFtime = 3;
int flag;
int period = 100;  //frecuencia de actualización del loop 100ms


//---------------------------------motor
unsigned long tiempoM = 0;
unsigned long t1 = 0;
bool pasos1 = LOW;  //para los pasos del motor
byte pasos = 8;

////////////////////////Variables PID///////////////////////
float distancia = 0.0;
float elapsedTime, timePrev;
float error_previo_distancia, error_distancia;
float distancia2 = 0.0;
float elapsedTime2, timePrev2;
float error_previo_distancia2, error_distancia2;
//---------------------TEMP-------------
int Vo, lastVo;
float R1 = 20000;              // resistencia fija del divisor de tension 
float logR2, R2, TEMPERATURA;
float c1 = 2.114990448e-03, c2 = 0.3832381228e-04, c3 = 5.228061052e-07;
//----------------------------------
byte pinMosfet = 6;

float setpoint = 143;

float kp = 0.09;
float ki = 1;
float kd = 8;

float PID_p, PID_i, PID_d, PID_total;

float error_tmp;


void setup()
{
  
Serial.begin(250000);
pinMode(pinMosfet, OUTPUT);

  pinMode(pasos, OUTPUT);
  pinMode(13, OUTPUT);

}

void loop()
{  
  lastVo = Vo;

  Vo = analogRead(A4);      // lectura de A0
  if (Vo == 1023){
    Vo = lastVo;
  }

  
  R2 = R1 * (1023.0 / (float)Vo - 1.0); // conversion de tension a resistencia
  logR2 = log(R2);      // logaritmo de R2 necesario para ecuacion
  TEMPERATURA = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));   // ecuacion S-H
  TEMPERATURA = TEMPERATURA - 273.15;   // Kelvin a Centigrados (Celsius)

//-------------------------------------------------------
  
  tiempo1++;
  flag =  ciclo1 * PID_p;

  
  if (tiempo1 > ciclo1){
    tiempo1 = 0;
  }

  
  if(tiempo1 > 0 && tiempo1 < flag){
   digitalWrite(pinMosfet,HIGH);
  }
    
  if(tiempo1 > flag && tiempo1 < ciclo1){
  digitalWrite(pinMosfet,LOW);
  }

  
   if (millis() > tiempo+period){
        tiempo = millis();    
    
    error_tmp = setpoint - TEMPERATURA;
    PID_p = kp*error_tmp;
  
    float dist_diference = error_distancia - error_previo_distancia;       
    PID_d = kd*((error_distancia - error_previo_distancia)/period);
      
    PID_i = PID_i + (ki * error_distancia);
     
    PID_total = PID_p + PID_i + PID_d;  
   
    error_previo_distancia = error_distancia;    
  }
//--------------------------------------
   if (millis() > tiempoI+periodI){
        tiempoI = millis();   
        Serial.println(TEMPERATURA);
        }   

//--------------------------------------
  int lectura = analogRead(A7);
  lectura = map(lectura, 0, 1023, 0, 30);
  tiempoM = millis();
      
  if( (tiempoM - t1) >= lectura ){
    t1 = tiempoM;
    pasos1 = !pasos1;
    
  }
  
 digitalWrite(pasos, pasos1);
 digitalWrite(13, pasos1);

}
