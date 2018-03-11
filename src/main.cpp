#include <WioLTEforArduino.h>
#include <WioLTEClient.h>
#include <PubSubClient.h>		// https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>
#include <ServoRS304.h>

#define BUTTON_PIN (WIOLTE_D38)

#define APN               "soracom.io"
#define USERNAME          "sora"
#define PASSWORD          "sora"

#define MQTT_SERVER_HOST  "beam.soracom.io"
#define MQTT_SERVER_PORT  (1883)

#define ID                "myDevice"
#define OUT_TOPIC         "devices/myDevice/messages/events/"
#define IN_TOPIC          "devices/myDevice/messages/devicebound/#"
#define DIRECT_METHOD_TOPIC "$iothub/methods/POST/#"
#define DIRECT_METHOD_REPLY_TOPIC ""

#define INTERVAL          (60000)

const char NO_ITEM = 0xA0;
const char HAVE_ITEM = 0xB0;
const char TAKE_ON_ITEM = 0xC0;

const char LIFT_DOWN = 0x00;
const char LIFT_UP = 0x01;
const char LIFTING = 0x02;
const char OVER_TIME = 0x04;

const char IRQ_PUSHED_BUTTON = 0x01;
const char IRQ_STATUS_CHECKE = 0x02;
const char IRQ_ACCEPT_DIRECT_METHOD = 0x04;
const char IRQ_SERVO_STATUS = 0x04;
const char IRQ_OVER_TIME = 0x08;

char imsi[16];
char irq_event =0x00;
char status;
char prevStatus;
char status_change_event = 0x00;
const int INTARVAL = 500;
const char servoId = 0x01;

const int loadAverageBufferSize = 20;
int loadCounter = 0;

//IoT Hub Direct method.
const char *directMethodName;
const char *requestNumber;

WioLTE wio;
WioLTEClient WioClient(&wio);
PubSubClient MqttClient;
ServoController servoController(Serial);


void setStatus(char item, char lift){
    prevStatus = status;
    status = (item & 0xF0) | (lift & 0x0F);
    if(prevStatus != status){
        status_change_event = 0x01;
    }
}

void liftUp(){
    SerialUSB.println("Lift up!");
    wio.LedSetRGB(0x00,0x00,0xA0);
    setStatus((status & 0xF0), LIFTING);
    if((status & 0xF0) == NO_ITEM){
        servoController.setMaxTorque(servoId, 0x28); //0x14 is 20
        servoController.restartServo(servoId);
    }
    servoController.turnOnTorque(servoId);
    servoController.moveServo(servoId, 700, 50);
    delay(500);
//    servoController.turnOffTorque(servoId);
    setStatus((status & 0xF0), LIFT_UP);
    SerialUSB.println("End Lift UP!");
}

void liftDown(){
    SerialUSB.println("Lift down!");
    wio.LedSetRGB(0x00,0x00,0x00);
    servoController.setMaxTorque(servoId, 0x64); //0x14 is 20
    servoController.restartServo(servoId);
    servoController.turnOnTorque(servoId);
    servoController.moveServo(servoId, 0, 50);
    delay(500);
    servoController.turnOffTorque(servoId);
    setStatus((status & 0xF0), LIFT_DOWN);
    SerialUSB.println("End Lift DOWN!");
}

void liftToggle(){
    if((status & 0x0F) == LIFT_DOWN){
        liftUp();
    }else if((status & 0x0F) == LIFT_UP){
        liftDown();
    }
}

void statusControll(){
    if(status == (OVER_TIME | HAVE_ITEM)){
        wio.LedSetRGB(0xff,0x00,0x00);
        SerialUSB.println("Please take something");
    }else if(status == (OVER_TIME | NO_ITEM)){
        wio.LedSetRGB(0x00,0x00,0xA0);
        liftDown();
    }else if(status == (NO_ITEM | LIFT_UP)){
        wio.LedSetRGB(0x00,0xA0,0x00);
        if(prevStatus == (HAVE_ITEM | LIFT_UP)){
            wio.LedSetRGB(0x00,0x00,0x00);
            liftDown();
        }
    }else if(status == (HAVE_ITEM | LIFT_UP)){
        if(prevStatus == (NO_ITEM | LIFT_UP)){
            wio.LedSetRGB(0x00,0x00,0x00);
            liftDown();
        }
    }else if(status == (NO_ITEM | LIFT_DOWN)){
        wio.LedSetRGB(0x00,0x00,0x00);
    }
}

/*int getLoadBuffer(){
    int sum = 0;
    for(int index = 0; index < loadAverageBufferSize; index++){
        sum += servoController.getCurrentServoLoad(servoId);
        delay(10);
    }
    return sum / loadAverageBufferSize;
}*/

void checkLiftState(){
    short currentAngle = servoController.getCurrentAngle(servoId);
    if(currentAngle > 500){
        if((status & 0x0F) != OVER_TIME){
            setStatus((status & 0xF0) , LIFT_UP);
        }
    }else{
        setStatus((status & 0xF0), LIFT_DOWN);
    }
}

/*void checkHaveItem(){
    short currentLoad=0;
    if((status & 0x0F) == LIFT_DOWN){
        servoController.turnOnTorque(servoId);
        delay(500);
        currentLoad = servoController.getCurrentServoLoad(servoId);
        delay(100);
        servoController.turnOffTorque(servoId);
        return ;
    }else{
        currentLoad = servoController.getCurrentServoLoad(servoId);
    }

    if(currentLoad > 30){
        setStatus(HAVE_ITEM, (status & 0x0F));
    }else{
        setStatus(NO_ITEM, (status & 0x0F));
    }

}*/

/*void alertCounter(int load, const int loopTime, const int waitTime){
    const int threshold = 30;
    int waitCount = waitTime / loopTime;

    if(status == (HAVE_ITEM | LIFT_UP)){
        loadCounter++;
    }

    if(loadCounter > waitCount){
        loadCounter = 0;
        setStatus((status & 0xF0), OVER_TIME);
    }

}*/

void getServoStatus(short* angle, short* load, float* volt){
    *angle = servoController.getCurrentAngle(servoId);
    *load = servoController.getCurrentServoLoad(servoId);
    *volt = servoController.getCurrentVoltage(servoId);
}

String getStatusString(char status){
    String result = "\"itemStatus\":";
    if((status & 0xF0) == 0xA0){
        result += "\"NO ITEM\",";
    }else if((status & 0xF0) == 0xB0){
        result += "\"HAVE ITEM\",";
    }else{
        result += "\"ERROR\",";
    }
    result +="\"liftStatus\":";
    if((status & 0x0F) == LIFT_DOWN){
        result += "\"DOWN\"";
    }else if((status & 0x0F) == LIFT_UP){
        result += "\"UP\"";
    }else if((status & 0x0F) == OVER_TIME){
        result += "\"OVERTIME\"";
    }else{
        result += "\"ERROR\"";
    }
    return result;

}

void callback(char* topic, byte* payload, unsigned int length) {
  SerialUSB.println("Subscribe:");
  SerialUSB.println(topic);
  char *token;
  token = strtok(topic,"/=");
  token = strtok(NULL,"/=");
  token = strtok(NULL,"/=");
  token = strtok(NULL,"/=");
  char *method = "upLift";
  if(strcmp(method, token) == 0 ){
      liftUp();
  }
  token = strtok(NULL,"/=");
  token = strtok(NULL,"/=");
  char data[100];
  sprintf(data, "$iothub/methods/res/1/?$rid=%s", token);
  MqttClient.publish(OUT_TOPIC, data);
 }

void buttonCheck(){
  //when Timer3 compare 1
  int buttonState = digitalRead(BUTTON_PIN);
  if(buttonState == 1){
    irq_event = (irq_event | IRQ_PUSHED_BUTTON);
  }
}

void updateStatus(){
    //when Timer3 compare 2
    irq_event = (irq_event | IRQ_STATUS_CHECKE);
}


void checkOverTime(){
  //when Timer3 compare 2
  int  previusTime = 2;
  int waitTime = 10;
  if(status == (HAVE_ITEM | LIFT_UP)){
        loadCounter++;
  }else{
    loadCounter = 0;
  }
  if(loadCounter > (waitTime / previusTime)){
        loadCounter = 0;
        setStatus((status & 0xF0), OVER_TIME);
        irq_event = (irq_event | IRQ_OVER_TIME);
  }
}

void changeDeviceMode(){
  irq_event = (irq_event | IRQ_SERVO_STATUS);
}

void setup() {
    wio.Init();
    SerialUSB.println("######## Start up device");
    status = LIFT_DOWN || NO_ITEM;

    SerialUSB.println("### Power supply ON.");
    wio.PowerSupplyLTE(true);
    delay(5000);

    SerialUSB.println("### Turn on or reset.");
    if (!wio.TurnOnOrReset()) {
      SerialUSB.println("### ERROR! ###");
      return;
    }

    SerialUSB.println("### Connecting to \""APN"\".");
    delay(5000);
    if (!wio.Activate(APN, USERNAME, PASSWORD)) {
      SerialUSB.println("### ERROR! ###");
      return;
    }
    //Get IMSI from soracom metadata service.
    //curl -s http://metadata.soracom.io/v1/subscriber
    //{"imsi":"44010XXXXXXXXXX","msisdn":"81XXXXXXXXXX","ipAddress":"10.XXX.XXX.XX","apn":"soracom.io","type":"s1.standard","groupId":"XXXXXXXXXXXXXXXXXXXXXXXXXX","createdAt":1444126013510,"lastModifiedAt":1452656126137,"expiredAt":null,"terminationEnabled":false,"status":"active","tags":{"name":"noriyuki01"},"sessionStatus":{"lastUpdatedAt":1452656126137,"imei":"XXXXXXXXXXXX","location":null,"ueIpAddress":"10.XXX.XXX.XXX","dnsServers":["100.127.0.53","100.127.1.53"],"online":true},"speedClass":"s1.standard","moduleType":"nano","plan":0,"expiryTime":null,"createdTime":1444126013510,"operatorId":"OPXXXXXXXX","lastModifiedTime":1452656126137}
    int metadataJsonLength = 1000;//Please forgot change to pool size of StaticJsonBuffer.
    char metadataJsonStr[metadataJsonLength];
    if(!wio.HttpGet("http://metadata.soracom.io/v1/subscriber",metadataJsonStr,metadataJsonLength)){
        exit(-1);
    }else{
        SerialUSB.println(metadataJsonStr);
        
        StaticJsonBuffer<1000> jsonBuffer;
        JsonObject &root= jsonBuffer.parseObject(metadataJsonStr);
        if (!root.success()) {
         SerialUSB.println("parseObject() failed");
         return;
        }
        const char* tmp = root["imsi"];
        strcpy(imsi,tmp);//440103197795859
        SerialUSB.println(imsi);
    }

    SerialUSB.println("### Connecting to MQTT server \""MQTT_SERVER_HOST"\"");
    MqttClient.setServer(MQTT_SERVER_HOST, MQTT_SERVER_PORT);
    MqttClient.setCallback(callback);
    MqttClient.setClient(WioClient);
    if (!MqttClient.connect(imsi)) {//IMSI Nuber is deviceId for IoTHub
      SerialUSB.println("### ERROR! ###");
      wio.LedSetRGB(0xFF,0x00,0x00);
      return;
    }
    MqttClient.subscribe(DIRECT_METHOD_TOPIC);
    servoController.begin();

    SerialUSB.println("### Initial GPIO");
    pinMode(BUTTON_PIN, INPUT);

    Timer3.pause();                               
    Timer3.setPrescaleFactor(10000);              // Timer clock 168MHz/10k = 16800Hz 
    Timer3.setOverflow(33600);                    //Counter is 33600 = 2 sec.
    Timer3.setCompare(1, 8400);
    Timer3.attachInterrupt(1, buttonCheck);      // コンパレータ1にて割り込み発生
    Timer3.setCompare(2, 33600);
    Timer3.attachInterrupt(2, updateStatus);      // コンパレータ2にて割り込み発生
    Timer3.setCompare(3, 16800);
    Timer3.attachInterrupt(3, checkOverTime);
    Timer3.setCompare(4, 25200);
    Timer3.attachInterrupt(4, changeDeviceMode);
    Timer3.refresh();                             // タイマーの更新
    Timer3.resume();                            // タイマースタート
    

}



void loop() {
    // put your main code here, to run repeatedly:
  SerialUSB.println("loop");

  
  unsigned long next = millis();
  while (millis() < next + INTERVAL)
  {
    MqttClient.loop();
    //TODO In final change to switch from if.
    if(irq_event & IRQ_PUSHED_BUTTON){
        SerialUSB.println("PUSH_DOWN_BUTTON");
        liftToggle();      
    }

    if(irq_event & IRQ_STATUS_CHECKE){
        SerialUSB.println("CHECK_STATUS");
        checkLiftState();    
        short currentLoad = 0.0;  
        if(((status & 0x0F) == LIFT_UP) || ((status & 0x0F) == OVER_TIME)){
          currentLoad = servoController.getCurrentServoLoad(servoId);
          SerialUSB.print("load: ");SerialUSB.println(currentLoad);
          if(currentLoad > 35){
            setStatus(HAVE_ITEM, (status & 0x0F));
          }else{
            setStatus(NO_ITEM, (status & 0x0F));
          }
        }
        char data[300];
        if(status_change_event == 0x01){
            //SerialUSB.print("status: ");SerialUSB.println(getStatusString(status));
            String sta = getStatusString(status);
//                        {"deviceId":"myDevice", "liftStatus":"DOWN", "itemStatus":"In", "weight":90.5}
            sprintf(data, "{\"imsi\":\"%s\", %s, \"weight\":%d}", imsi, sta.c_str(), currentLoad);
            SerialUSB.println(data);
            MqttClient.publish(OUT_TOPIC, data);
            status_change_event = 0x00;
        }
    }

    if(irq_event & IRQ_SERVO_STATUS){
      if(status == (OVER_TIME | HAVE_ITEM)){
        wio.LedSetRGB(0xff,0x00,0x00);
        SerialUSB.println("Please take something");
      }else if(status == (OVER_TIME | NO_ITEM)){
        wio.LedSetRGB(0x00,0x00,0xA0);
        liftDown();
      }else if(status == (NO_ITEM | LIFT_UP)){
        wio.LedSetRGB(0x00,0xA0,0x00);
        if(prevStatus == (HAVE_ITEM | LIFT_UP)){
          wio.LedSetRGB(0x00,0x00,0x00);
          liftDown();
        }
      }else if(status == (HAVE_ITEM | LIFT_UP)){
        if(prevStatus == (NO_ITEM | LIFT_UP)){
          wio.LedSetRGB(0x00,0x00,0x00);
          liftDown();
        }
      }else if(status == (NO_ITEM | LIFT_DOWN)){
        wio.LedSetRGB(0x00,0x00,0x00);
      }
    }

    if(irq_event & IRQ_OVER_TIME){
      SerialUSB.println("OVER_TIME");
      if(status == (OVER_TIME | HAVE_ITEM)){
        wio.LedSetRGB(0xff,0x00,0x00);
        SerialUSB.println("Please take something");
      }else if(status == (OVER_TIME | NO_ITEM)){
        wio.LedSetRGB(0x00,0x00,0xA0);
        liftDown();
      }
    }

    irq_event = 0x00;
    
  }

}