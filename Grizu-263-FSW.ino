#include <Wire.h>
#include<EEPROM.h>
#include <TinyGPS++.h>
#include "I2Cdev.h"
#include <DS1307new.h>
#include <Kalman.h> 
#include <Timer.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MPU9255.h>
#define RESTRICT_PITCH
double R1=150000; //pil gerilimi ölçmek için 
double R2=100000;// gerilim bölücü devredeki direnç değerlerine göre belirlenmiştir.
int deger=0;
float vout;
float vin;

int denet=0;//zaman sayacı için kullanılmıştır.
int ddenet=0;//zaman sayacını kontrol emek için
const int buttonPin = 8;   
const int ledPin = 9;      
int buttonState = 0;   
  
double alt2=0;//uydu hareketinin aşağı yönlü mü  yoksa yukarı 
double alt1=0;//yönlü olduğunu kontrol etmek için
float durum=0;
int koontrol=0;
int checkk=0;

int photo_count=0;
int oku;
int ID=3944;//Team ID
double referanc_alt=0;//yükseklik için bu yarışma yerine göre değiştirilecek
double alt=0;
int sat=0;//GPS uydu sayısı
double lant=0;
double longt=0;
double g_alti=0;
int t_sn=0;//GPS verileri için
int t_dk=0;
int t_sa=0;
int state=0;//sistemin hangi aşamada olduğunu yer istasyonuna bildirmede kullanılır.
String giden;
unsigned long timer1;
unsigned long eski;
unsigned long eski1;

int counter=0;
int packet_co=1;;
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t TmpMPU;
double gyroXangle, gyroYangle, gyroZangle;
double compAngleX, compAngleY, compAngleZ;
double kalAngleX, kalAngleY, kalAngleZ;
uint32_t timer;
double hsabit=0;
uint16_t startAddr = 0x0000;            // Start address to store in the NV-RAM
uint16_t lastAddr;                      // new address for storing in NV-RAM
uint16_t TimeIsSet = 0xaa55;            // Helper that time must not set again
static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gp(RXPin, TXPin);

MPU9255 MPU;
Adafruit_BMP280 bmp;
void GPs();

//*************************************************************************************************************************

void setup() {
  #if ARDUINO >= 157
    Wire.setClock(400000UL);
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2;
  #endif
  Serial.begin(19200);
  while (!Serial) {;}
  bmp.begin();
  Wire.begin();
  gp.begin(GPSBaud);//GPS için
  MPU.init();
  MPU.set_acc_scale(4);
  MPU.set_gyro_scale(4);
  RTC.setRAM(0, (uint8_t *)&startAddr, sizeof(uint16_t));// Store startAddr in NV-RAM address 0x08 

  RTC.getRAM(54, (uint8_t *)&TimeIsSet, sizeof(uint16_t));
  
  if (TimeIsSet != 0xaa55)
  {
    RTC.stopClock();
        
    RTC.fillByYMD(2018,5,20);
    RTC.fillByHMS(00,00,0);//saat eğer ayarlı değilse bu şekilde ayarla
    
    RTC.setTime();
    TimeIsSet = 0xaa55;
    RTC.setRAM(54, (uint8_t *)&TimeIsSet, sizeof(uint16_t));
    RTC.startClock();
  }
  else
  {
    RTC.getTime();
  }

  RTC.ctrl = 0x00;                      // 0x00=disable SQW pin, 0x10=1Hz,
                                        // 0x11=4096Hz, 0x12=8192Hz, 0x13=32768Hz
                                        
pinMode(4,OUTPUT);
digitalWrite(4, HIGH); //KAMERA İÇİN
delay(3000);
digitalWrite(4, LOW); 

delay(3000);
pinMode(ledPin, OUTPUT);
pinMode(buttonPin, INPUT);
pinMode(7, OUTPUT);//BUZZER
pinMode(5, OUTPUT);// 1. ayrılma,
pinMode(6, OUTPUT);// 2. ayrılma
digitalWrite(9, HIGH); 

packet_co=EEprom_read(40,41);
counter=EEprom_read(22,23);
photo_count=EEprom_read(50,51);
referanc_alt=EEprom_read(60,61);

                                        
}
//******************************************************************************************************
void loop() {
  timer1=millis();
  ReadMPU();
  giden="";
  giden+=ID;
  giden+=",";
  ReadDS3231();//Mission time
  giden+=packet_co;
  giden+=",";
  hsabit=bmp.readPressure()/100;
  alt=bmp.readAltitude(1015)-referanc_alt;
  giden+=alt;
  giden+=",";
  giden+=bmp.readPressure();//Pa
  giden+=",";
  giden+=bmp.readTemperature();//*C
  giden+=",";
  giden+=voltage();
  giden+=",";
  GPs();
  giden+=kalAngleX;
  giden+=",";
  giden+=kalAngleY;
  giden+=",";
  giden+=kalAngleZ;
  giden+=",";
  giden+=state;
  giden+=",";
  giden+=photo_count;
  if(timer1-eski>=960){//bu koşul 960 ms de bir aktif olarak telemetrinin gönderim hızını 1HZ e yakın tutmak için kullanılıyor.
    eski=timer1;       
    packet_co=packet_co+1;
    EEprom_Save(40,41,packet_co);
    Serial.println(giden);
    if(ddenet==1){ //istenilen yerlerde ddenet kontrol edilerek denet değişkeni zaman sayacı olarak görev yapıyor.
      denet++;     // farkettiyseniz koşulumuz birdiğer koşul olan timer1-eski<=960 koşulu içinde.
    
       }
    }
      flight(alt,3);
      buttonn();
      camera();
}
//**********************************************************************************************************
void ReadMPU(){//Kalman filtresi kullanıarak tilt x,y,z nin hesaplanması
  MPU.read_acc();
  MPU.read_gyro();
  MPU.read_mag();
  MPU.read_temp();
  
  accX=MPU.ax;
  accY=MPU.ay;
  accZ=MPU.az;
  TmpMPU=MPU.tmp;
  gyroX=MPU.gx;
  gyroY=MPU.gy;
  gyroZ=MPU.gz;
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    double yaw = atan(accZ/sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;
  #else
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    double yaw = atan(accZ/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  #endif
    double gyroXrate = gyroX / 131.0;
    double gyroYrate = gyroY / 131.0;
    double gyroZrate = gyroZ / 131.0;
  #ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)){
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  }
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
  if(abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
  #else
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    }
    else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
      kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
    if(abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate;
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
  #endif
  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
  if(gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if(gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if(gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;
}
//******************************************************************************************
void ReadDS3231(){//RTC data
  RTC.getTime(); // Saat ve Tarih verilerini al
  if (RTC.hour < 10)                  
  {
    giden+="0";
    giden+=RTC.hour,DEC;
  } 
  else
  {
  giden+=RTC.hour, DEC;
  }
   giden+=":";
  if (RTC.minute < 10)
  {
    giden+="0";
    giden+=RTC.minute, DEC;
  }
  else
  {
    giden+=RTC.minute, DEC;
  }
  giden+=":";
  if (RTC.second < 10) 
  {
    giden+="0";
    giden+=RTC.second, DEC;
  }
  else
  {
    giden+=RTC.second, DEC;
  }
 
  giden+=",";
  
  }
//*******************************************************************************************
void GPs(){
while (gp.available() > 0){
gps.encode(gp.read());
if (gps.time.isUpdated()){
t_sa=gps.time.hour(); // Hour (0-23) (u8)
t_dk=gps.time.minute();// Minute (0-59) (u8)
t_sn=gps.time.second(); // Second (0-59) (u8)
}
 if (gps.location.isUpdated())
  {
 
lant=gps.location.lat(), 6;// Latitude in degrees (double)
 
longt=gps.location.lng(), 6;// Longitude in degrees (double)
  }
   
  if (gps.altitude.isUpdated()){
g_alti=gps.altitude.meters(); // Altitude in meters (double)
  }
  
 if (  gps.satellites.isUpdated()){
sat=gps.satellites.value(); // Number of satellites in use (u32)
}
 
}
giden+=t_sa;
giden+="-";
giden+=t_dk;
giden+="-";
giden+=t_sn;
giden+=",";
giden+=String(lant,6);
giden+=",";
giden+=String(longt,6);
giden+=",";
giden+=g_alti;
giden+=",";
giden+=sat;
giden+=",";


}


//********************************************************************************************
 int EEprom_Save(int x,int y,int deger){//gerekli verileri  EEPROM'a kaydedeliyor.
 byte a=lowByte(deger);
  byte b=highByte(deger);
  EEPROM.update(x,a);
  EEPROM.update(y,b);
  //byte c=EEPROM.read(x);
  //byte d=EEPROM.read(y);
  //int e=c+(d<<8);
  //return e;
 
 }
 //*******************************************************************************************
int EEprom_read(int k,int u){//işlemcinin sıfırlanması dumunda verilerin EEPROM daki adreslerinden okunarak kurtarılmaı için
 byte f=EEPROM.read(k);
 byte g=EEPROM.read(u);
 int degerin=f+(g<<8);
 return degerin;
 
 }

 //********************************************************************************************
  
 void ayrilma1(){//yükseklik 500 ms de bir konrol edilerek uydunun düşüşe geçtiği anda ısı kalkanı açılır.
  alt1=bmp.readAltitude(1015);
  delay(500);
  alt2=bmp.readAltitude(1015);
  durum=alt2-alt1;
  
      if(durum<=0){
      checkk=1;
      }
   
   if(koontrol==0){
     if(checkk==1){
     burning(1,3);
   //Serial.println("HIGH");
     }
     }
     
    /*  if(durum>0){
    //digitalWrite(7,LOW);
   //Serial.println("LOW");

    
   }
    */
   
 }
 //**************************************************************************************************************
 
 
 
 void burning(int hangi,int bekleme){
  //0 ohm direnç yardımıyla misinanın yakılarak ayrılma ve açılma işlemlerinin gerçekleştirilmesi
  if(hangi==1){

  if(denet<1){
       ddenet=1;
      }
 
  if(ddenet==1){
    digitalWrite(5, HIGH); 
  //Serial.println("1.ayrılmastart"); 
 // Serial.println(denet); 

  }
   if(denet>=bekleme){//sistemde gecikme nedeniyle veri kaybı yaşanmaması için delay fonksiyonu yerine bu şekilde bir kontrol sağlanmıştır.
    digitalWrite(5, LOW);
    //Serial.println("1.ayrılmafinish"); 
    state=2;
    ddenet=0;
    denet=0;
    koontrol=1;
    }
    
 
   }
  if(hangi==2){
    if(denet<1){
       ddenet=1;
      }
   if(ddenet==1){
    digitalWrite(6, HIGH);  
  //Serial.println("2.ayrılmastart"); 
  //Serial.println(denet);
  }  
  if(denet>=bekleme){//sistemde gecikme nedeniyle veri kaybı yaşanmaması için delay fonksiyonu yerine bu şekilde bir kontrol sağlanmıştır.
    digitalWrite(5, LOW);
    //Serial.println("2.ayrılmafinish"); 
    counter=2;
    ddenet=0;
    denet=0;
    }
  }
 }
 //********************************************************************************************
 void zil(){
      while(1){
        
         digitalWrite(7, HIGH);  

      }

 }
 //********************************************************************************************
 float voltage(){
  
  deger=analogRead(A3);
  vout = (deger* 5.0) / 1024.0; 
  vin = (vout / R2)*(R2+R1);
  return vin;
  
}
//*********************************************************************************************
void camera(){//bu fonsiyon kullandığımız kameranın açma kapama ve fotoğraf çekmek için kullanılan butonunu kontrol etmek için kullanılıyor.
  if(timer1-eski1>=100){
         digitalWrite(4,LOW);  
         //Serial.println("HIGH");
        
    }
     if(timer1-eski1<100){
         digitalWrite(4,HIGH);  
         //Serial.println("LOW");

    }
     if(timer1-eski1>=1100){
      eski1=timer1;
      photo_count++;
      //Serial.println(photo_count);
      EEprom_Save(50,51,photo_count);
    }
  
  
}

//*********************************************************************************************
void EEPROM_clear(){//gerekli durumlarda EEpromdaki verilerin silinmesi için kullanılır.

for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.update(i, 0);//eğer i. adress '0' değilse '0' olarak değiştir.(EEPROM.update)
    delay(20);
    digitalWrite(13, HIGH);//EEPROM Sıfırlandığında led yak.

  }  
}
//***********************************************************************************************

void buttonn(){//uydu rokete konulmadan öbce butona basılır ve uçuş öncesi ayarlamalar buton fonksiyonu 
//yardımıyla gerekli ayarlamalar yyapılır
 buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    digitalWrite(9, LOW); 
    EEPROM_clear();
    delay(1000);
    referanc_alt=bmp.readAltitude(1015);//Uydunun yerden yüksekliğini bulmak için uçuş öncesi bulunulan nokta referans alınır.
    photo_count=0;
    packet_co=0;
    counter=0;
    EEprom_Save(60,61,referanc_alt);
   // EEprom_Save(40,41,packet_co);
   // EEprom_Save(50,51,photo_count);
    //EEprom_Save(22,23,counter);
    digitalWrite(9, HIGH); 

  } 
    
}
//***********************************************************************************************

 
 void flight(int yukseklk,int burn_wait){
  
  if(counter==0){//uçuş görevleri counter değişkeni ile kontrol ediliyor
  
  if(alt<5){
    state=0;

     }
  if(alt>=5){
    state=1;
  }
  }
  if(yukseklk>=450){
    if(counter==0){
      counter=1;
      EEprom_Save(22,23,counter);
    }
   
    
  }
  if(counter==1){
      
      ayrilma1();//1. ayrılma (ısı kalkanının açılması)
    

      

    
    if(yukseklk<=330){
       burning(2,burn_wait);//2. ayrılma probun ısı kalkanınından ayrılması
       EEprom_Save(22,23,counter);
       state=3;

    }
     }
     
   if(counter==2){
    state=4; 
    if(yukseklk<10){
        state=5;
       // camera();

      }

      if(yukseklk<5){
        zil();
       
      }
      
   }
 }


 

  


