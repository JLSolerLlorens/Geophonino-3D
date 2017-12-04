/*Geophonino is an Arduino-based seismic recorder for vertical geophones.  
The computer programming consists of: 
GeophoninoHVSR.ino: The Arduino Sketch.
GuiGeophoninoHVSR: The user interface developed by using Processing software.

Last update: 01/02/2017

Email contact: jl.soler@ua.es, juanjo@dfists.ua.es*/

#include <SPI.h>
#include <SD.h>
#include <DueTimer.h>

/*Initialization of variables*/
#define BUF_SZ   400

int duration = 0;
int gain = 0; //1=x0.5,    2=x1,    3=x2
int gLtb = 0; //0=x0, 1=x1, 2=x2, 3=x5, 4=x10, 5=x20, 6=x50, 7=x100
int srsel=0;  // 0="Sr 10ms, 100-Hz" 1= "Sr 4ms, 250-Hz",     2= "Sr 2ms, 500-Hz",     3= "Sr 1ms, 1000-Hz"

String cabSrStr="";
String cabGainStr="";
String cabGLtbStr="";
String cabTimeRecStr="";
float addtime=0;
bool firsttime=true; 

char nomFile[]="";
bool endacq=true; //endacq=false during acquisition

volatile unsigned long initime=0;
volatile unsigned long timeISR=0;

const int chipSelect = 4; 

File dataFile;

int cont=1000; //milliseconds

volatile bool buffAct=0;

volatile bool clearbuf0=0;
volatile bool clearbuf1=0;

volatile int sample0 = 0;
volatile int sample1 = 0;
volatile int timesam0[2000] = {0};
volatile int timesam1[2000] = {0};
volatile uint16_t datab0 [2000] [3]={0,0,0};
volatile uint16_t datab1 [2000] [3]={0,0,0};

int req_index = 0;              // index into req buffer
char separator[]="&";
char ser_req[BUF_SZ] = {0}; 

volatile bool sta=true; //state change for clk_write ouput

volatile bool G0=false;
volatile bool G1=false;
volatile bool G2=false;
 
int pinspeaker=9;

/*Interrupt Service Routine to generate clock signal for MAX7404 filter*/
void clkwrite()
{
  digitalWrite(3,sta);
  sta=!sta;
}


/* Interrupt Service Routine to acquire data*/
void isrADC(){
  if(buffAct)
  {
    timesam1[sample1]=(millis()-initime);
    datab1[sample1][0]=analogRead(0); 
    datab1[sample1][1]=analogRead(1); 
    datab1[sample1][2]=analogRead(2);
    sample1++;
    if(sample1>500 & sample0==0) 
    {
      clearbuf1=1; 
      buffAct=!buffAct;
    }
  }
  else
  {
    timesam0[sample0]=(millis()-initime);
    datab0[sample0][0]=analogRead(0); 
    datab0[sample0][1]=analogRead(1); 
    datab0[sample0][2]=analogRead(2);
    sample0++;
    if(sample0>500 & sample1==0) 
    {
      clearbuf0=1; 
      buffAct=!buffAct;
    }
  }
}

/*Setup block*/
void setup() {
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(pinspeaker,OUTPUT);
  
  Serial.begin(115200);
  waitGuiContact();
}

/*Loop block*/
void loop() 
{
  
  //**********************¿Should clean Buffer0?*********************
  if(!endacq & clearbuf0)
  {
    clearbuf0=0;
    for (int i = 0; i < sample0; i++) {
      dataFile.println(String(timesam0[i])+","+String(datab0[i][0])+","+String(datab0[i][1])+","+String(datab0[i][2]));
    }
    sample0=0;
  }  
  
  //¿Should clean Buffer1?
  else if(!endacq & clearbuf1)
  {
    clearbuf1=0;
    for (int i = 0; i < sample1; i++) {
      dataFile.println(String(timesam1[i])+","+String(datab1[i][0])+","+String(datab1[i][1])+","+String(datab1[i][2]));
    }
    sample1=0;
  }
  //**************** ¿Should clean Buffers? ********************
  
  
  //send a signal to GUI when one second of data is acquired
  if(millis()-initime>cont & !endacq)
  {
      Serial.println("adq_now_ok"+String(((millis()-initime)/1000))); 
      cont=cont+1000;
  }

  //***************************** END ACQUISITION *****************************
  if(((millis()-initime)>(duration*1000+addtime)) & !endacq) 
  {
    Timer3.stop();
    Timer4.stop();
    
    Serial.println("Timer parado");
    
     if(!endacq & sample0>0)
    {
      buffAct=!buffAct;
      clearbuf0=0;
      for (int i = 0; i < sample0; i++) {
        dataFile.println(String(timesam0[i])+","+String(datab0[i][0])+","+String(datab0[i][1])+","+String(datab0[i][2]));
      }
      sample0=0;
    }
    if(!endacq & sample1>0)
    {
      buffAct=!buffAct;
      clearbuf1=0;
      for (int i = 0; i < sample1; i++) {
        dataFile.println(String(timesam1[i])+","+String(datab1[i][0])+","+String(datab1[i][1])+","+String(datab1[i][2]));
      }
      sample1=0;
    }
    
    endacq=true;   
    dataFile.close();
    clearbuf0=0;
    clearbuf1=0;
    sample1=0;
    sample0=0;
    buffAct=0;
    cont=1000;
    Serial.println("adq_end");
    endacq=true;
    buzzer(2500);
  }
  
   if (Serial.available() > 0) // If data is available to read.
   { 
    dataSerialInput();
   }   
}

/*Program is waiting in a loop while GUI don't send a contact message*/
void waitGuiContact() 
{
  while (Serial.available() <= 0) {
  Serial.println("A");   // Waiting
  delay(300);
  }
}

/*dataSerialInput() Is called when data is received by serial port*/
void dataSerialInput()
{
  char val = Serial.read(); // read it and store it in val
  if (val != '\n')
  {
    ser_req[req_index] = val;          // save Serial request character
    req_index++;
  }
  else
  {
    Serial.println(ser_req);
    /************************     CONFIG ADQ     *****************************/
    /* GUI sends data acquisition parameters*/
    
    if(StrContains(ser_req, "conf_adq"))
    {
      Serial.println("modo conf_adq");
      Serial.println(ser_req);
        
      String gainStr="",timeRecStr="",sRateStr="", nomFichStr="", overW="", gLtbStr="";
      
      char sfind[]="nomF";
      nomFichStr=StrValueExtract(ser_req,sfind,separator);
      Serial.println(nomFichStr);
      if(nomFichStr=="-1"){nomFichStr="Default.dat";}
      
      nomFichStr.toCharArray(nomFile, sizeof(nomFichStr));
      
      strcpy(sfind,"overW");
      overW=StrValueExtract(ser_req,sfind,separator);
      Serial.println(overW);
      
      if(existFile() & overW=="N")
      {
        Serial.println("conf_adq_filexist");
      }
      else //If filename don't exists continue searching configuration parameters
      {          
        strcpy(sfind,"Gain");
        gainStr=StrValueExtract(ser_req,sfind,separator);
        if(gainStr=="-1"){gainStr=="1";}  
        
        Serial.println(gainStr);
        gain=gainStr.toInt();
       
        switch(gain)  
        {  
          case 1:
            cabGainStr="0.5";
          break;
          case 2:
            cabGainStr="1";
          break;
          case 3:
            cabGainStr="2";
          break;
        }

        strcpy(sfind,"tR");
        timeRecStr=StrValueExtract(ser_req,sfind,separator);
        cabTimeRecStr=timeRecStr;
        if(timeRecStr=="-1"){timeRecStr=="1";}  
        Serial.println(timeRecStr);
        duration=timeRecStr.toInt();
        
        strcpy(sfind,"sR");
        sRateStr=StrValueExtract(ser_req,sfind,separator);
        if(sRateStr=="-1"){sRateStr=="1";}  
        Serial.println(sRateStr);
        srsel=sRateStr.toInt();
        switch(srsel)  //0="10ms, 100-Hz", 1= "4ms, 250-Hz",     2= "Sr 2ms, 500-Hz",     3= "Sr 1ms, 1000-Hz"
        {
          case 0:
            cabSrStr="10";
            Timer4.attachInterrupt(clkwrite).setFrequency(8000).start(); //Clk=4 KHz    fc=40 Hz
          break;
          case 1:
            cabSrStr="4";
            Timer4.attachInterrupt(clkwrite).setFrequency(20000).start(); //Clk=10 KHz    fc=100 Hz
          break;
          case 2:
            cabSrStr="2";
            Timer4.attachInterrupt(clkwrite).setFrequency(40000).start(); //Clk=20 KHz    fc=200 Hz
          break;
          case 3:
            cabSrStr="1";
            Timer4.attachInterrupt(clkwrite).setFrequency(80000).start(); //Clk=80000    fc=400 Hz
          break;
        }
        Serial.println(cabSrStr);
        
        strcpy(sfind,"GLtb");
        gLtbStr=StrValueExtract(ser_req,sfind,separator);
        if(gLtbStr=="-1"){gLtbStr=="0";}  
        Serial.println(gLtbStr);
        gLtb=gLtbStr.toInt();
        switch(gLtb)  
        {  
          case 0:
            cabGLtbStr="0";
          break;
          case 1:
            cabGLtbStr="1";
          break;
          case 2:
            cabGLtbStr="2";
          break;
          case 3:
            cabGLtbStr="5";

          break;
          case 4:
            cabGLtbStr="10";
          break;
          case 5:
            cabGLtbStr="20";
          break;
          case 6:
            cabGLtbStr="50";
          break;
          case 7:
            cabGLtbStr="100";
          break;
        }
        

        Serial.println(cabSrStr);
        
        Serial.println("conf_adq_ok");
      }
  
    }//END if ConfiqADQ
    
  /************************     ADQ NOW     *****************************/
  /* GUI requests that acquisition start now*/
          
    if(StrContains(ser_req, "adq_now"))
    {
      configAnalogInput();
      if(openFile())
      {
        Serial.println("adq_now_ok");
        launchAcquisition(); 
      }
    }
    
  /************************     GET F NOW     *****************************/
  /* GUI requests a data file*/
  
    if(StrContains(ser_req,"getF_now"))
    {
      File dataF = SD.open(nomFile,FILE_READ);
      if (dataF) 
      {  
         Serial.println("sendF_size"+String(dataF.size()));
         delay(100);
         Serial.println("sendF_now");       
         delay(100);  
         while (dataF.position() < dataF.size()) 
         {   
          Serial.write(dataF.read());  
          Serial.flush();
          delay(0.1); 
         }   
         dataF.close();  
         Serial.println("sendF_now_end"); 
      }    
      else 
      {   
         Serial.println("error sending File");
         Serial.println(nomFile);   
      }        
    }
    
  /************************     GET F KO     *****************************/
    if(StrContains(ser_req,"getF_ko"))
    {
      Serial.println("Get File Cancelled by user");
    }
    req_index=0;
    StrClear(ser_req, BUF_SZ);
  }//close Else save serial request
} //close dataSerialINput() method

void launchAcquisition()
{
    //srsel=->  0= "10ms, 100-Hz",  1= "4ms, 250-Hz",  2= "Sr 2ms, 500-Hz",  3= "Sr 1ms, 1000-Hz";
    switch(srsel)
    {
      case(0):
        delay(16000);
        initime=millis();
        Timer3.attachInterrupt(isrADC).start(10000); // Every 10 ms -> 100 samples per second
        endacq=false;	
        addtime=11; //milliseconds
      break;
      case(1):
        delay(16000);
        initime=millis();
        Timer3.attachInterrupt(isrADC).start(4000); // Every 4 ms -> 250 samples per second
        endacq=false;	
        addtime=5; //millisecons
      break;
      case(2):
        delay(16000);
        initime=millis();
        Timer3.attachInterrupt(isrADC).start(2000); // Every 2 ms -> 500 samples per second
        endacq=false;
        addtime=3;
      break;
      case(3):
        delay(16000);
        initime=millis();
        Timer3.attachInterrupt(isrADC).start(1000); // Every 1 ms -> 1000 samples per second
        endacq=false;
        addtime=2;
      break;
    }   
}

bool existFile()
{
  if (!SD.begin(chipSelect) & firsttime) {
      Serial.println("Card failed, or not present");
      return(false);
    }
    Serial.println("card initialized.");
    firsttime=false;
    if (SD.exists(nomFile)) {
      return true;
      Serial.println("File exist"); 
    }
    else
    {
      return false;
    }
}

bool openFile()
{
  if (!SD.begin(chipSelect) & firsttime) {
      Serial.println("Card failed, or not present");
      return(false);
    }
    Serial.println("card initialized.");
    firsttime=false;
    if (SD.exists(nomFile)) {
      SD.remove(nomFile);
      Serial.println("File Deleted"); 
    }
  dataFile = SD.open(nomFile, FILE_WRITE);  
  dataFile.println("Gain,"+cabGainStr+",Duration,"+cabTimeRecStr+",Samplerate,"+cabSrStr+",LtbAmplification,"+cabGLtbStr);
  return(dataFile);
}

void configAnalogInput()
{
  analogReadResolution(12);
  //gain -> 1=x0.5,    2=x1,    3=x2
  switch(gain)
  {
    case(1):
      ADC->ADC_CGR = 0x00000000; //set gain = 0.5 all chanels
    break;
    case(2):
      ADC->ADC_CGR=0x15555555; //set gain = 1 all chanels
    break;
    case(3):
     ADC->ADC_CGR=0xFFFFFFFF; //Set Gain = 2 on channel A0-A7
    break;
  }
  
  switch(gLtb)
  {
    case(0):  //000
     G2=false;  G1=false;  G0=false;     
    break;
    case(1):  //001
     G2=false;  G1=false;  G0=true;     
    break;
    case(2):  //010
     G2=false;  G1=true;  G0=false;     
    break;
    case(3):  //011
     G2=false;  G1=true;  G0=true;     
    break;
    case(4):  //100
     G2=true;  G1=false;  G0=false;     
    break;
    case(5):  //101
     G2=true;  G1=false;  G0=true;     
    break;
    case(6):  //110
     G2=true;  G1=true;  G0=false;     
    break;
    case(7):  //111
     G2=true;  G1=true;  G0=true;     
    break;
  }
  
  digitalWrite(7,G2); digitalWrite(6,G1); digitalWrite(5,G0);
  
  // Set up the configuration of each ADC channel
  REG_ADC_MR = (REG_ADC_MR & 0xFF0FFFFF) | 0x00B00000;
  ADC->ADC_WPMR &= ~(1<<ADC_WPMR_WPEN); //unlock register
  
  //ADC->ADC_COR = 0xFFFF0000;    //Uncomment to set fully differential mode to all chanels
  ADC->ADC_COR = 0x00000000;      //Set single ended mode to all chanels
  
  /*set STARTUP time ADC to a smaller number,*/
  unsigned long reg;
  REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;
  reg = REG_ADC_MR;
  
}

// Searches for the string sfind in the string str
// Returns 1 string found
// Returns 0 string not found
char StrContains(char *str, char *sfind)
{
    char found = 0;
    char index = 0;
    char len;

    len = strlen(str);
    
    if (strlen(sfind) > len) {
        return 0;
    }
    while (index < len) {
        if (str[index] == sfind[found]) {
            found++;
            if (strlen(sfind) == found) {
                return 1;
            }
        }
        else {
            found = 0;
        }
        index++;
    }

    return 0;
}

void StrClear(char *str, char length)
{
    for (int i = 0; i < length; i++) {
        str[i] = 0;
    }
}

/* StrValueExtract return variables that was found in a string
str string to analize
sfind variable name
separator separator usually &
str example:"conf_adq&nomF=nomfile&overW=N&Gain=2&tR=15&sR=1"
*/
String StrValueExtract(char *str, char *sfind, char *separator)
{
    unsigned int found = 0;
    unsigned int index = 0;
    int len;
    char value[] = "";
    unsigned int indValue=0;
    bool founded=false;
    len = strlen(str);
    if (strlen(sfind) > len) {
        Serial.println("ERROR, string finded is longer than original string");
    }
    while (index < len) {
        if ((str[index] == sfind[found]) | founded) {
            found++;
            if ((strlen(sfind) == found & str[index+1]=='=') | founded) {
                founded=true;
                if(str[index+2]!=separator[0] & (index+2<strlen(str)))
                {
                  value[indValue]=str[index+2];
                  indValue++;
                }
                else
                {
                  value[indValue]='\0';
                  return String(value);
                }
            }
        }
        else {
            found = 0;
        }
        index++;
    }
    Serial.println("NotFound");
    value={'-1\0'}; //not found
    return String(value);
}

void buzzer(int loopcount)
{
  for(int i=0;i<loopcount;i++)
  {   
      
      digitalWrite(pinspeaker,HIGH);
      delayMicroseconds(1136);
      digitalWrite(pinspeaker,LOW);
      delayMicroseconds(1136);
  }
}
