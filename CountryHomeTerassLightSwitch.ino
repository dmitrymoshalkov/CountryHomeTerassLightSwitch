#include <Wire.h>
#include <PortExpander_I2C.h>
#include <MySensor.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <avr/wdt.h>
#include <SimpleTimer.h>



//#define NDEBUG                        // enable local debugging information

#define SKETCH_NAME "Terrass light switch"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"
#define NODE_ID 22 //or AUTO to let controller assign     


#define RELAY1_CHILD_ID 30

#define BUTTON1_CHILD_ID 40

#define BUTTON1DELAYED_CHILD_ID 45

#define RELAY1_STATUS_CHILD_ID 50

#define TEMP1_CHILD_ID	60

#define REBOOT_CHILD_ID                       100
#define RECHECK_SENSOR_VALUES                 101 
#define LOCAL_SWITCHING_CHILD_ID              106
#define NIGHTMODE_CHILD_ID                    105

#define BUTTON1_PIN	6


#define LED1_PIN	A0


#define RELAY1_PIN	5

#define NOCONTROLLER_MODE_PIN	7


#define ONE_WIRE_BUS              4      // Pin where dallase sensor is connected 

#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

#define TEMPCHECK_TIME 15000
#define TEMPCHECK_SEND_TIME 120000
#define GWSTATUSCHECK_TIME 300000


const unsigned long debounceTime = 10;  // milliseconds
unsigned long switch1PressTime;  // when the switch last changed state


unsigned long switch1Time;  // when the switch last changed state



byte oldSwitch1State = HIGH;  // assume switch open because of pull-up resistor


boolean bRelay1State=false;



boolean bRelay1DelayMessageSent=false;


boolean localSwitching = true;

#define RADIO_RESET_DELAY_TIME 50 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения
#define DATASEND_DELAY  10

boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;

boolean boolRecheckSensorValues = false;

OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass the oneWire reference to Dallas Temperature. 
unsigned long previousTempMillis=0;
unsigned long previousTempSendMillis=0;
float lastTemp1 = -1;

unsigned long previousStatMillis = 0;
boolean bGatewayPresent = true;

byte bNoControllerMode = HIGH;
boolean bNightMode = false;


MySensor sensor_node;

MyMessage msgButton1(BUTTON1_CHILD_ID, V_LIGHT);


MyMessage msgRelay1Status(RELAY1_STATUS_CHILD_ID, V_STATUS);


MyMessage msgRelay1(RELAY1_CHILD_ID, V_STATUS);



MyMessage msgRelay1Delayed(BUTTON1DELAYED_CHILD_ID, V_STATUS);



MyMessage msgLocalSwitching(LOCAL_SWITCHING_CHILD_ID, V_STATUS);

MyMessage msgTemp1(TEMP1_CHILD_ID, V_TEMP);




void setup() {

 Serial.begin(115200);

 pinMode(A0, OUTPUT);

  pinMode(BUTTON1_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(BUTTON1_PIN,HIGH);

  pinMode(NOCONTROLLER_MODE_PIN,INPUT);
  // Activate internal pull-up
  digitalWrite(NOCONTROLLER_MODE_PIN,HIGH);


   bNoControllerMode = digitalRead(NOCONTROLLER_MODE_PIN);


   if (bNoControllerMode != LOW)
   {

        for (int i=0; i<=12; i++)
    {
      digitalWrite(A0, HIGH);
      delay(70);
      digitalWrite(A0, LOW);    
      delay(70);      
    }

  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);


  sensor_node.begin(incomingMessage, NODE_ID, false);

  sensor_node.wait(RADIO_RESET_DELAY_TIME);
  sensor_node.sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER"."SKETCH_MINOR_VER);

    	//relays
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY1_CHILD_ID, S_LIGHT);   

      	//relays status
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAY1_STATUS_CHILD_ID, S_BINARY);   

        //buttons S_LIGHT
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(BUTTON1_PIN, S_LIGHT);  

        //temperature sensors
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(TEMP1_CHILD_ID, S_TEMP);    

        //reboot sensor command
    sensor_node.wait(RADIO_RESET_DELAY_TIME);
    sensor_node.present(REBOOT_CHILD_ID, S_BINARY); //, "Reboot node sensor", true); 

    //reget sensor values
    sensor_node.wait(RADIO_RESET_DELAY_TIME);
  	sensor_node.present(RECHECK_SENSOR_VALUES, S_LIGHT);     

    //local switching
    sensor_node.wait(RADIO_RESET_DELAY_TIME);
  	sensor_node.present(LOCAL_SWITCHING_CHILD_ID, S_BINARY);    

  	sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(RELAY1_CHILD_ID, V_LIGHT);
   
    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(LOCAL_SWITCHING_CHILD_ID, V_STATUS);  

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(NIGHTMODE_CHILD_ID, V_TRIPPED);     
}
else
{

        for (int i=0; i<=3; i++)
    {
      digitalWrite(A0, HIGH);
      delay(250);
      digitalWrite(A0, LOW);    
      delay(250);      
    }

}

  	  //Enable watchdog timer
  wdt_enable(WDTO_8S);



        #ifdef NDEBUG
          Serial.print(F("End setup "));
        #endif


}

void loop() {

 byte switchState = digitalRead(NOCONTROLLER_MODE_PIN);

  if ( switchState != bNoControllerMode )
  {

  		#ifdef NDEBUG
          Serial.print("Controller mode ");
          Serial.println(switchState);          
        #endif
  		
  		bNoControllerMode = switchState;
  		wdt_enable(WDTO_30MS);
        while(1) {};
  	
  }

checkGatewayStatus();


chechButton1();


checkTemperature();

   if (bNoControllerMode != LOW)
   {
		sensor_node.process();
	}

if (boolRecheckSensorValues)
{

  boolRecheckSensorValues = false;
  resendRelayStatus();
}


    //reset watchdog timer
    wdt_reset();    
}


void switchRelayON_OFF( byte RelayPin, byte Status )
{
	digitalWrite(RelayPin, Status ); //switch relay

}


void incomingMessage(const MyMessage &message) {

  if (message.isAck())
  {
    gotAck = true;
    return;
  }

    if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true && strlen(message.getString())>0 ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }
     
     if (message.type==V_LIGHT && strlen(message.getString())>0 && message.sensor == RELAY1_CHILD_ID) 
     {



     		   bRelay1State = message.getBool();
     		   switchRelayON_OFF( RELAY1_PIN, message.getBool()?RELAY_ON:RELAY_OFF );

     		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay1Status.set(message.getBool()?"1":"0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

     
     }

    if ( message.sensor == LOCAL_SWITCHING_CHILD_ID  && strlen(message.getString())>0) {
         
         
         localSwitching = message.getBool()?false:true;

     		   			//Отсылаем состояние переключателя
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgLocalSwitching.set(localSwitching?"0":"1"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

         

     }

    if ( message.sensor == RECHECK_SENSOR_VALUES && strlen(message.getString())>0) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;


         }

     }

    if ( message.sensor == NIGHTMODE_CHILD_ID  && strlen(message.getString())>0 ) {
         
         if (message.getBool() == true)
         {
            digitalWrite(LED1_PIN, HIGH);
            bNightMode = true;
         }
         else
         {
         	digitalWrite(LED1_PIN, LOW);
            bNightMode = false;

            
         }

     }

        return;      
} 


void chechButton1 ()
{

  int iCountStat = -1;

  byte switchState = digitalRead (BUTTON1_PIN);
  

  if (switchState != oldSwitch1State)
    {

    // debounce
    if (millis () - switch1PressTime >= debounceTime)
       {
       //switchTime = switchPressTime;

       if (switchState == LOW)
          {

          	digitalWrite(LED1_PIN, HIGH);

      if (bNoControllerMode != LOW && bGatewayPresent)
      {

      		if ( !localSwitching )  
      		{
		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton1.set("1"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;

                iCountStat =  iCount;                 

            }
       }



		  #ifdef NDEBUG	
          Serial.println ("Switch closed.");
          #endif

          }  // end if switchState is LOW
        else
          {

          #ifdef NDEBUG
          Serial.print ("Switch press time: ");
          Serial.println (millis () - switch1PressTime);   
          Serial.println ("Switch opened.");
          #endif 


            if ( localSwitching && !bRelay1DelayMessageSent )  
            {
			    if ( !bRelay1State ) //msgRelay1
			    {

			    	bRelay1State = true;
			    	switchRelayON_OFF( RELAY1_PIN, RELAY_ON );

			    	if (bNoControllerMode != LOW && bGatewayPresent)
			    	{
			    		//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay1Status.set("1"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;		
                      iCountStat =  iCount;                        
			    	}
		    	

			    }	
			    else
			    {

			    	bRelay1State = false;
			    	switchRelayON_OFF( RELAY1_PIN, RELAY_OFF );       

			    	if (bNoControllerMode != LOW && bGatewayPresent )
			    	{
					    //Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgRelay1Status.set("0"), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	
                      iCountStat =  iCount;                        
			    	}
	    	  			    	
			    }
			}


          bRelay1DelayMessageSent = false;	

      if (bNoControllerMode != LOW && bGatewayPresent)
      {
      		if (!localSwitching)
      		{
		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
      
    	            sensor_node.send(msgButton1.set("0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;
                iCountStat =  iCountStat + iCount;  

            }   
	  }


            if ( !bNightMode )
            {
            	digitalWrite(LED1_PIN, LOW);
            }
            else
            {
            	digitalWrite(LED1_PIN, HIGH);
            }

          }  // end if switchState is HIGH

       switch1PressTime = millis ();  // when we closed the switch 
       oldSwitch1State =  switchState;  // remember for next time 

          setGatewayStatus(iCountStat); // if no answer from gateway go to nosend mode

           
       }  // end if debounce time up
        
    }  // end of state change
    else
    {
    	if (switchState == LOW && !bRelay1DelayMessageSent)
    	{
          if (millis () - switch1PressTime >=2000)
          {
          	if ( bRelay1State )
          	{
   
			}
			else
			{
				//send delayed status message 				
			}

			          	digitalWrite(LED1_PIN, LOW);

	   if (bNoControllerMode != LOW && bGatewayPresent)
	   {
		    //Отсылаем нажатие кнопки с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
    	             // Send in the new temperature                  
    	            sensor_node.send(msgRelay1Delayed.set(bRelay1State?"1":"0"), true);
                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;  
        }        
			bRelay1DelayMessageSent = true;
          }

    	}

    }

}

int resendRelayStatus()
{



   			//Отсылаем состояние реле с подтверждением получения
        iCount = MESSAGE_ACK_RETRY_COUNT;

          while( !gotAck && iCount > 0 )
            {
  
	            sensor_node.send(msgRelay1Status.set(bRelay1State?"1":"0"), true);
                sensor_node.wait(RADIO_RESET_DELAY_TIME);
              iCount--;
             }

            gotAck = false;	
  			  		     		
  return iCount;
}


void checkTemperature()
{


    unsigned long currentTempMillis = millis();
    if((currentTempMillis - previousTempMillis ) > TEMPCHECK_TIME ) {
        // Save the current millis
        previousTempMillis = currentTempMillis;

// Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  sensor_node.wait(conversionTime);

 float temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(0) * 10.)) / 10.;



         if (temperature != lastTemp1 && temperature != -127.00 && temperature != 85.00 ) {

          		#ifdef NDEBUG                
                Serial.print ("Temp: ");
          	    Serial.println (temperature); 
          	    #endif

          	    	   if (bNoControllerMode != LOW)
          	    	   {
          	    	   	currentTempMillis = millis();
    					if((currentTempMillis - previousTempSendMillis ) > TEMPCHECK_SEND_TIME ) {
        				// Save the current millis
        				previousTempSendMillis = currentTempMillis;

     		   			//Отсылаем состояние реле с подтверждением получения
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgTemp1.set(temperature,2), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	   

                      setGatewayStatus(iCount);  

			            }
			        	}
			                if ( temperature >= 50 )
			                {
			                	//switch off all relays
     		  					 bRelay1State = false;
     		   					 switchRelayON_OFF( RELAY1_PIN, RELAY_OFF );	
                     resendRelayStatus();		                	
		   					      		   					 
			                }     		

            lastTemp1 = temperature;
        	} 




      }
}

void setGatewayStatus(int iRert)
{

if ( bNoControllerMode != LOW)
{
    if ( iRert == 0 && bGatewayPresent )
    {
        bGatewayPresent = false;

    }
    else if ( iRert > 0 && !bGatewayPresent )
    {

        bGatewayPresent = true;
        resendRelayStatus();

    }
}
}

void checkGatewayStatus()
{

if ( bNoControllerMode != LOW )
{
    unsigned long currentStatMillis = millis();
    if((currentStatMillis - previousStatMillis ) > GWSTATUSCHECK_TIME ) 
      {
        // Save the current millis
        previousStatMillis = currentStatMillis;



      if ( resendRelayStatus() > 0 )
      {

          bGatewayPresent = true;

      }
      else
      {

          bGatewayPresent = false;

      }

      }


}
}


