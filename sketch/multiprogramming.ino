#include "analogWrite.h"
#include <Arduino_JSON.h>
#include <time.h>
#include <WiFi.h>
#include <HTTPClient.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
QueueHandle_t xQueue_sensor;
QueueHandle_t xQueue_target;

struct SensorData
{
    int timestamp[100];
    float pos[100];
    int count;
    int target;
};

int IN1 = 18;
int IN2 = 23;
int PWM = 2;
int ENCA = 32;
int ENCB = 34;

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

const char* ssid = "pratyanshu-Inspiron-5570";
const char* password = "hope1985";

String get_target_url = "http://esw-onem2m.iiit.ac.in:443/~/in-cse/in-name/Team-4/Node-1/queue_data?rcn=4";
String post_sensordata_url = "http://esw-onem2m.iiit.ac.in:443/~/in-cse/in-name/Team-4/Node-1/sensor_data";

uint wifi_delay = 5000;
uint lastTime = 0;
#define PID_TIMER 5000

HTTPClient http;


void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    pinMode(ENCA,INPUT);
    pinMode(ENCB,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

    analogWriteResolution(PWM, 8);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);

    WiFi.begin(ssid, password);
    Serial.println("Connecting");
    while(WiFi.status() != WL_CONNECTED)
        delay(500);
    Serial.print("\nConnected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());

    xQueue_sensor = xQueueCreate( 5, sizeof( struct SensorData ) );
    xQueue_target = xQueueCreate( 5, sizeof( int ));


    xTaskCreatePinnedToCore(Task1code,"Task1",4000,NULL,5,&Task1,0);                         
    delay(1000); 

}

void Task1code( void * parameter ){
    Serial.print("Task1 is running on core ");
    Serial.println(xPortGetCoreID());

    String tq_rn[5] = {"", "", "", "", ""};

    for(;;){
        if(WiFi.status()== WL_CONNECTED)
        {

            // Your Domain name with URL path or IP address with path
            http.begin(get_target_url.c_str());

            http.addHeader("X-M2M-Origin", "KK382AITgR:WPN0JcTeJd");
            http.addHeader("Accept", "application/json");

            // Send HTTP GET request
            int httpResponseCode = http.GET();

            if (httpResponseCode>0) 
            {
                Serial.print("HTTP Response code: ");
                Serial.println(httpResponseCode);
                String payload = http.getString();

                JSONVar myObject = JSON.parse(payload);
                if (JSON.typeof(myObject) == "undefined") 
                {
                    Serial.println("Parsing input failed!");

                }
                else
                {

                    Serial.print("JSON.typeof(myObject) = ");
                    Serial.println(JSON.typeof(myObject)); // prints: object

                    int len = (int)myObject["m2m:cnt"]["cni"];
                    for(int i = 0; i < len; i++)
                    {
                        String rn = (const char*)myObject["m2m:cnt"]["m2m:cin"][i]["rn"];
                        int j = 0;
                        for(j = 0; j < 5; j++)
                            if(rn == tq_rn[j])
                                break;
                        if(j == 5)
                        {

                            String target_string = (const char*)myObject["m2m:cnt"]["m2m:cin"][i]["con"];
                            int target = target_string.toInt();
                            xQueueSend(xQueue_target, &target, portMAX_DELAY);


                            struct SensorData sensor_data;
                            while(true)
                            {
                                if (xQueueReceive( xQueue_sensor, &sensor_data, portMAX_DELAY ) == pdPASS)
                                {
                                    Serial.print( "Received = ");
                                    Serial.println(sensor_data.count);
                                    if(sensor_data.count == -1)
                                        break;
                                    else
                                        sendSensorData(sensor_data);
                                }
                            }
                        }
                    }

                    for(int i = 0; i < len; i++)
                        tq_rn[i] = (const char*)myObject["m2m:cnt"]["m2m:cin"][i]["rn"];

                }        
            }
            else {
                Serial.print("Error code: ");
                Serial.println(httpResponseCode);
            }
            // Free resources
            http.end();
        }
        else {
            Serial.println("WiFi Disconnected");
        }
        lastTime = millis();
    }
}

void loop(){
    int target;
    if (xQueueReceive( xQueue_target, &target, portMAX_DELAY ) == pdPASS)
    {
        Serial.print( "Received = ");
        Serial.println(target);
        

        PID_control(target);
        setMotor(0,0,PWM,IN1,IN2); 
    }
}

void PID_control(int target)
{
    
    uint startTime = millis();
    uint lastTime = millis();
    struct SensorData sensor_data;
    sensor_data.count = 0;
    sensor_data.target = target;
    target = target * 46.0 * 11.0 / 360.0;
    while(millis()- startTime < PID_TIMER){

        // PID constants
        float kp = 6;
        float kd = 0.025;
        float ki = 0.5;

        // time difference
        long currT = micros();
        float deltaT = ((float) (currT - prevT))/( 1.0e6 );
        prevT = currT;

        // Read the positionL
        int pos = 0; 
        noInterrupts(); // disable interrupts temporarily while reading
        pos = posi;
        interrupts(); // turn interrupts back on

        // error
        float e = target - pos;

        // derivative
        float dedt = (e-eprev)/(deltaT);

        // integral
        if(dedt == 0)
            eintegral = eintegral + e*deltaT;
        if(eintegral > 255)
            eintegral = 255;
        if (eintegral < -255)
            eintegral = -255;

        // control signal
        float u = kp*e + kd*dedt + ki*eintegral;

        // motor power
        float pwr = fabs(u);
        if( pwr > 255 ){
            pwr = 255;
        }

        // motor direction
        int dir = 1;
        if(u<0){
            dir = -1;
        }

        // signal the motor
        setMotor(dir,pwr,PWM,IN1,IN2);


        // store previous error
        eprev = e;

        Serial.print(target * 0.71146245059);
        Serial.print(" ");
        Serial.print(pos * 0.71146245059);
        sensor_data.pos[sensor_data.count] = pos * 0.71146245059;
        sensor_data.timestamp[sensor_data.count] = millis()- startTime;
        sensor_data.count += 1;
        //  sensor_data += (String)(millis()- startTime) + ":" + (String)(pos * 0.71146245059) + ",";
        //  Serial.print(" ");
        //  Serial.print(e);
        //  Serial.print(" ");
        //  Serial.print(dedt);
        //  Serial.print(" ");
        //  Serial.print(eintegral);
        //  Serial.print(" ");
        //  Serial.print(u);
        Serial.println();

        if(millis() - lastTime > 500 || sensor_data.count >= 100)
        {
            lastTime = millis();
            //      setMotor(0,0,PWM,IN1,IN2);
            //      sendSensorData(sensor_data);
            Serial.println("Sending sensor data");
            xQueueSend( xQueue_sensor, &sensor_data, portMAX_DELAY );
            sensor_data.count = 0;
        }
    }
    xQueueSend( xQueue_sensor, &sensor_data, portMAX_DELAY );
    sensor_data.count = -1;
    xQueueSend( xQueue_sensor, &sensor_data, portMAX_DELAY );
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
    if(dir == 1){
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
    }
    else if(dir == -1){
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);
    }
    else{
        digitalWrite(in1,LOW);
        digitalWrite(in2,LOW);
    }  
    analogWrite(pwm,pwmVal);
}

void readEncoder(){
    int b = digitalRead(ENCB);
    if(b > 0){
        posi++;
    }
    else{
        posi--;
    }
}

void sendSensorData(struct SensorData sensor_data_st)
{
    Serial.println(millis());
    String sensor_data = "";
    for(int i = 0; i < sensor_data_st.count; i++)
        sensor_data += (String)sensor_data_st.timestamp[i] + ":" + (String)sensor_data_st.pos[i] + ":" + (String)sensor_data_st.target + ",";

    http.begin(post_sensordata_url.c_str());

    // Your Domain name with URL path or IP address with path

    Serial.println(millis());

    http.addHeader("X-M2M-Origin", "KK382AITgR:WPN0JcTeJd");
    http.addHeader("Content-Type", "application/json;ty=4");

    String ciRepresentation =
        "{\"m2m:cin\": {"
        "\"con\":\"" + sensor_data + "\""
        "}}";
    // Send HTTP GET request
    int httpResponseCode = http.POST(ciRepresentation);
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
//    Serial.println(ciRepresentation);
    Serial.println(millis());

    // Free resources
    http.end();
    Serial.println(millis());
}
