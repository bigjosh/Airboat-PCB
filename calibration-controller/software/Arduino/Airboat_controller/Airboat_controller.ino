

void setup() {
  // put your setup code here, to run once:

  pinMode( 8 , OUTPUT );

}

#define ON()    digitalWrite(8,LOW)
#define OFF()   digitalWrite(8,HIGH)

// Returns 0 or 1

uint8_t parity( uint8_t b ) {

  uint8_t p=0;

  while (b) {

    if (b & 1) p^=1;
    b >>=1;
  }

  return(p);
}


// Commands:
// duty: 001 xxxx dddddddd p
// freq: 010 ssss tttttttt p

int sendCommand( uint8_t command, uint8_t a, uint8_t b ) {

  // Sanity check field lengths

  if (command > 0b111 )   return(1);
  if (a       > 0b1111 )  return(2);

  // Did you know that the xor of two bytes together has the same partity of the two bytes seporately?

  unsigned packet = ( command << 13U ) | ( a << 9 ) | ( b << 1 ) | parity( command ^ a ^ b );
   
    
  OFF();      // Must start with off becuase otherwise the battery could be full and thus CIP not active

  delay(250);    // Initial off lead in 

  unsigned  bitmask = 1U<<15;

  // Create the top of the first sync pulse. 
  // This has to be long enough to get the recievers attention and have them ready to see the 1st sync fall

  ON();
  _delay_us(10000);   // With the 2000us below we wil get a full 22ms before the first drop
                      // Since each main loop on the unit takes ~13ms, we need this to be long enough to 
                      // get the data decoders attention and have it ready and waiting for the falling edge, but
                      // Anything more than 16ms will be read as charging
  
  while (bitmask) {

    // Sync pulse       
    ON();
    _delay_us(3000);  
    OFF();              // We sync to this trailing edge
                        // Trailing is better becase the CIP responds almost instantly 
                        // when the voltage drops.

    _delay_us(1000);     // The trigger valley.                     

   // Next comes the actual data bit window. It must be 2000us long.
   // The first 1ms is rise time (if the bit is 1), then the next 1ms is sample window.
   // Sample targets the center of the window, 2500us after the falling sync edge (1500us after the rise)

    if (packet & bitmask) {

      // Do nothing on most bits - the next sync pulse will be sampled and read as high. Saves 2ms on 1 bits.

      if (bitmask==1) {       // Are we on the final bit?

          ON();
          _delay_us(3000);    // we have to send ourselves
          OFF();
      
      }
            
    } else {

      // stay off for 3000us. 
      _delay_us(3000);

    }
    
    bitmask >>= 1;
    
  }

  // we actually do not need to do anything here. we are always guaranteed to be low when we get here.

   return(0);
  
}



void loop() {

  sendCommand( 3 , 0 ,  );
  delay(500);
    
}


