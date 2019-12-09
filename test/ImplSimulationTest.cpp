#include <OneLimb.h>
#include <Arduino.h>
#include <Chrono.h>

//** sd card libraries, used for storing data in the sd card. 
#include <SD.h>
#include <SPI.h>
#ifndef __MK66FX1M0__
#error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif
#define pi 3.141508


Chrono wait_feedback;
Chrono control_time(Chrono::MICROS);
Chrono samplingvar(Chrono::MICROS);
uint8_t nodeid = 0x02;
String input_string = "";
uint8_t nodesid[4] = {0x2, 0x2 ,0x3, 0x4};

double torqueRef = 0;
int32_t motor_position = 0;
uint32_t anotherPos = 0;

unsigned long feedinterval = 100; 
int32_t statusT = 0;

unsigned long passed_reading = 0;
unsigned long samplingvarelapsed = 0;
unsigned long control_us = 0;
unsigned long counter = 0;

double data[101] = {-0.952761382221189,-1.06973607055769,-1.25721852940152,-1.15355547132598,-0.92210491098733,-0.944760154008175,-1.19862370810484,-1.45671749693296,-1.48960240089938,-1.41078382462277,-1.42473485572062,-1.58387988906206,-1.72057558702663,-1.72546758427493,-1.66209389493615,-1.64536016700365,-1.70165691833171,-1.76776497980077,-1.79044127754074,-1.77895680787845,-1.76852270589952,-1.77684756000496,-1.7968341802597,-1.81372979649838,-1.82146254989482,-1.82381126647805,-1.8265496158655,-1.83186424960997,-1.83843057192714,-1.84422155099742,-1.84847051535116,-1.85165950148915,-1.85454565505444,-1.85747533498893,-1.86035940991468,-1.86298493534544,-1.8652554749518,-1.86721865811025,-1.86897200187894,-1.87058024133185,-1.87205859711368,-1.87339945873812,-1.87460035432478,-1.87567247281946,-1.87663401203544,-1.87750124237766,-1.87828494626795,-1.87899197350501,-1.8796281469355,-1.88019979810419,-1.88071362965538,-1.88117594111741,-1.88159214573632,-1.88196680134148,-1.88230389400582,-1.8826070722807,-1.88287972325132,-1.8831249480186,-1.88334553130561,-1.88354395102469,-1.88372241911032,-1.88388292588624,-1.88402727094201,-1.88415708049877,-1.88427381928579,-1.88437880321587,-1.88447321408093,-1.8845581143979,-1.88463446050301,-1.88470311339114,-1.88476484787707,-1.88482036082811,-1.88487027884366,-1.88491516538271,-1.88495552722141,-1.88499182019864,-1.88502445431699,-1.8850537983105,-1.88508018377025,-1.88510390887882,-1.88512524178054,-1.88514442361113,-1.88516167121668,-1.88517717959571,-1.88519112409595,-1.88520366239215,-1.88521493626652,-1.8852250732103,-1.88523418786374,-1.88524238331071,-1.88524975224287,-1.88525637800676,-1.88526233554562,-1.88526769224638,-1.88527250870138,-1.88527683939328,-1.88528073331094,-1.88528423450325,-1.88528738257717,-1.88529021314555,-1.88529275822991};
const char* fileName = "datalog.txt";

OneLimb oneLimb;


//////**************************//////////////////////
void setup() {
    // Initiate Serial Communication
    Serial.begin(250000);
    delay(5000);
    Serial.println("Starting");
    // Activate the CAN bus device
    pinMode(28, OUTPUT);
    pinMode(35, OUTPUT);
    digitalWrite(28, LOW);
    digitalWrite(35, LOW);

    delay(500);

    LED(OFF);
    oneLimb.setZref(-0.4);
    wait_feedback.restart();

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        while (1);
    }
    Serial.println("card initialized.");
    SD.remove(fileName);
    delay(1000);
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile){
        dataFile.println("theta, zeta, eta, dtheta, dzeta, deta, z, zr, tau, ez");
        Serial.println("File ok");
        dataFile.close();
    } else {
        while(1){
            Serial.println("File error");
            delay(1000);
        }
    }

}
///////*************** MAIN PROGRAM ******//////////////////
void loop() {
    
    if (wait_feedback.hasPassed(feedinterval)) {
        wait_feedback.restart();
        Serial.print("loop | Motor position ticks: "); 
        Serial.print(motor_position);
        Serial.print(", "); 
        Serial.print(oneLimb.motorPosToDeg(motor_position));
        Serial.print(" degrees, ");
        Serial.print(oneLimb.motorPosToRad(motor_position));
        Serial.print(" radians, CAN delay:");
        Serial.print(passed_reading ); 
        Serial.print(", control_us: ");
        Serial.print(control_us);
        Serial.print(", Torque ref: ");
        Serial.print(torqueRef);
        Serial.print(", elapsed_comm: ");
        Serial.println(samplingvarelapsed);
        
        Serial.send_now();
    }

	if (control_time.hasPassed(CONTROLDELAY)) {
        
		control_us = control_time.elapsed();
		control_time.restart(); 

		// do some control 
		torqueRef = oneLimb.getTorque(data[counter]);
        oneLimb.writeToFile(fileName);
        ++counter;
    }
  

    while(counter > 100);
}