/****************************************************/
/* This is only one example of code structure       */
/* OFFCOURSE this code can be optimized, but        */
/* the idea is let it so simple to be easy catch    */
/* where can do changes and look to the results     */
/****************************************************/

#define VFD_in 8// If 0 write LCD, if 1 read of LCD
#define VFD_clk 9 // if 0 is a command, if 1 is a data0
#define VFD_stb 10 // Must be pulsed to LCD fetch data of bus
#define AdjustPins    PIND // before is C, but I'm use port C to VFC Controle signals

uint8_t numberDigits = 0b00000010;  //this define number of digits used! On this panel is 5 grids

/*Global Variables Declarations*/
unsigned char secU;
unsigned char secD;
unsigned char minU;
unsigned char minD;
unsigned char houU;
unsigned char houD;

unsigned char hours = 0;
unsigned char minutes = 0;
unsigned char seconds = 0;
unsigned char milisec = 0;
unsigned char points = 0;
unsigned char secs;

unsigned int letter[54] ={
  //This not respect the normal table for 7segm like "hgfedcba"  // 
  //Because the display is only 7 segments, is not possible represent all char's.
  ////...76543210....76543210.........
      0b01111110, 0b00000000, //A // 
      0b11111000, 0b00000000, //B // 
      0b11001010, 0b00000000, //C  // 
      0b11110100, 0b00000000, //D  // 
      0b11011010, 0b00000000, //E  // 
      0b01011010, 0b00000000, //F  // 
      0b00000000, 0b00000000, //G  // 
      0b00000000, 0b00000000, //H  // 
      0b00000000, 0b00000000, //I  // 
      0b00000000, 0b00000000, //J  // 
      0b00000000, 0b00000000, //K  // 
      0b00000000, 0b00000000, //L  // 
      0b00000000, 0b00000000, //M  // 
      0b00000000, 0b00000000, //N  // 
      0b00000000, 0b00000000, //O  // 
      0b00000000, 0b00000000, //P  // 
      0b00000000, 0b00000000, //Q  // 
      0b00000000, 0b00000000, //R  // 
      0b00000000, 0b00000000, //S  // 
      0b00000000, 0b00000000, //T  // 
      0b00000000, 0b00000000, //U  // 
      0b00000000, 0b00000000, //V  //
      0b00000000, 0b00000000, //X  // 
      0b00000000, 0b00000000, //Z  // 
      0b00000000, 0b00000000, //W  // 
      0b00000000, 0b00000000, //   // empty display
  };
unsigned int numbers[12][2] ={
  //This not respect the normal table for 7segm like "hgfedcba"  // 
  ////...76543210....76543210.........
      {0b11101110, 0b00000000}, //0  // 
      {0b00100100, 0b00000000}, //1  // 
      {0b11010110, 0b00000000}, //2  // 
      {0b10110110, 0b00000000}, //3  // 
      {0b00111100, 0b00000000}, //4  // 
      {0b10111010, 0b00000000}, //5  // 
      {0b11111010, 0b00000000}, //6  // 
      {0b00100110, 0b00000000}, //7  // 
      {0b11111110, 0b00000000}, //8  // 
      {0b00111110, 0b00000000}, //9  // 
      {0b00000000, 0b00000000}, //10 // empty display
  };
void pt6312_init(void){
  delayMicroseconds(200); //power_up delay
  // Note: Allways the first byte in the input data after the STB go to LOW is interpret as command!!!
  // Configure VFD display (grids)
  cmd_with_stb(numberDigits);// cmd1 6 grids 16 segm
  delayMicroseconds(5);
  // Write to memory display, increment address, normal operation
  cmd_with_stb(0b01000000);//(BIN(01000000));
  delayMicroseconds(5);
  // Address 00H - 15H ( total of 11*2Bytes=176 Bits)
  cmd_with_stb(0b11000000);//(BIN(01100110)); 
  delayMicroseconds(5);
  // set DIMM/PWM to value
  cmd_with_stb((0b10001000) | 7);//0 min - 7 max  )(0b01010000)
  delayMicroseconds(5);
}
void cmd_without_stb(unsigned char a){
  // send without stb
  unsigned char transmit = 7; //define our transmit pin
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  
  data=a;
  //This don't send the strobe signal, to be used in burst data send
   for (mask = 0b00000001; mask>0; mask <<= 1) { //iterate through bit mask
     digitalWrite(VFD_clk, LOW);
     if (data & mask){ // if bitwise AND resolves to true
        digitalWrite(VFD_in, HIGH);
     }
     else{ //if bitwise and resolves to false
       digitalWrite(VFD_in, LOW);
     }
    delayMicroseconds(5);
    digitalWrite(VFD_clk, HIGH);
    delayMicroseconds(5);
   }
   //digitalWrite(VFD_clk, LOW);
}
void cmd_with_stb(unsigned char a){
  // send with stb
  unsigned char transmit = 7; //define our transmit pin
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  
  data=a;
  
  //This send the strobe signal
  //Note: The first byte input at in after the STB go LOW is interpreted as a command!!!
  digitalWrite(VFD_stb, LOW);
  delayMicroseconds(1);
           for (mask = 00000001; mask>0; mask <<= 1) { //iterate through bit mask
             digitalWrite(VFD_clk, LOW);
             delayMicroseconds(1);
                   if (data & mask){ // if bitwise AND resolves to true
                      digitalWrite(VFD_in, HIGH);
                   }
                   else{ //if bitwise and resolves to false
                     digitalWrite(VFD_in, LOW);
                   }
            digitalWrite(VFD_clk, HIGH);
            delayMicroseconds(1);
           }
   digitalWrite(VFD_stb, HIGH);
   delayMicroseconds(1);
}
void test_VFD(void){
  /* 
  Here do a test for all segments of 6 grids
  each grid is controlled by a group of 2 bytes
  by these reason I'm send a burst of 2 bytes of
  data. The cycle for do a increment of 3 bytes on 
  the variable "i" on each test cycle of FOR.
  */
  // to test 7 grids is 7*3=21, the 8 gird result in 8*3=24.
 
  clear_VFD();
      
      digitalWrite(VFD_stb, LOW);
      delayMicroseconds(1);
      cmd_with_stb(numberDigits); // cmd 1 // 7 Grids & 15 Segments
      cmd_with_stb(0b01000000); // cmd 2 //Normal operation; Set pulse as 1/16
      
        digitalWrite(VFD_stb, LOW);
        delayMicroseconds(1);
        cmd_without_stb((0b11000000)); //cmd 3 wich define the start address (00H to 15H)
        
         for (uint8_t i = 0; i < 7 ; i++){ // test base to 16 segm and 6 grids
         cmd_without_stb(0b11111111); // Data to fill table 5*16 = 80 bits
         cmd_without_stb(0b11111111); // Data to fill table 5*16 = 80 bits
         }
 
      cmd_with_stb((0b10001000) | 7); //cmd 4
      digitalWrite(VFD_stb, HIGH);
      delay(1);
      delay(200);  
}
void test_VFD_chkGrids(void){
  /* 
  Here do a test for all segments of 5 grids
  each grid is controlled by a group of 2 bytes
  by these reason I'm send a burst of 2 bytes of
  data. The cycle for do a increment of 3 bytes on 
  the variable "i" on each test cycle of FOR.
  */
  // to test 6 grids is 6*3=18, the 8 grid result in 8*3=24.
 
  clear_VFD();
      
      digitalWrite(VFD_stb, LOW);
      delayMicroseconds(4);
      cmd_with_stb(numberDigits); // cmd 1 // 6 Grids & 16 Segments
      cmd_with_stb(0b01000000); // cmd 2 //Normal operation; Set pulse as 1/16
      
        digitalWrite(VFD_stb, LOW);
        delayMicroseconds(4);
        cmd_without_stb((0b11000000)); //cmd 3 wich define the start address (00H to 15H)
             for (int i = 0; i < 7 ; i++){ // test base to 15 segm and 7 grids
             cmd_without_stb(0b11111111); // Data to fill table 5*16 = 80 bits
             cmd_without_stb(0b11111111); // Data to fill table 5*16 = 80 bits
             }
          digitalWrite(VFD_stb, HIGH);
          delayMicroseconds(4);
      cmd_with_stb((0b10001000) | 7); //cmd 4
      
        delay(1);
        delay(100);
}
void wheels(uint8_t wL, uint8_t wH){
  /* 
  Here do a test for all segments of 5 grids
  each grid is controlled by a group of 2 bytes
  by these reason I'm send a burst of 2 bytes of
  data. The cycle for do a increment of 3 bytes on 
  the variable "i" on each test cycle of FOR.
  */
  short bitWheel = 0x0001;
  uint8_t wordL = 0x00;
  uint8_t wordH = 0x00;

  
      digitalWrite(VFD_stb, LOW);
      delayMicroseconds(4);
      cmd_with_stb(numberDigits); // cmd 1 // 6 Grids & 16 Segments
      cmd_with_stb(0b01000000); // cmd 2 //Normal operation; Set pulse as 1/16
      
      digitalWrite(VFD_stb, LOW);
      delayMicroseconds(4);
      cmd_without_stb((0b11000000)); //cmd 3 wich define the start address (00H to 15H)
             
      cmd_without_stb(wL); // Data to fill table 5*16 = 80 bits
      cmd_without_stb(wH); // Data to fill table 5*16 = 80 bits
            
      digitalWrite(VFD_stb, HIGH);
      delayMicroseconds(4);
      cmd_with_stb((0b10001000) | 7); //cmd 4
}
void test_VFD_grid(void){
  //Take account here I don't worry about corrections of segment "C"
  //because this is only to test the grid positions, and I skip grid "0"!!!
  uint8_t d = 0;    
      digitalWrite(VFD_stb, LOW);
      delay(15);
      cmd_with_stb(numberDigits); // cmd 1 // 7 Grids & 15 Segments
      cmd_with_stb(0b01000000); // cmd 2 //Normal operation; Set pulse as 1/16
      //
      digitalWrite(VFD_stb, LOW);
        delayMicroseconds(4);
      cmd_with_stb((0b11000000)); //cmd 3 wich define the start address (00H to 15H)
      for (uint8_t g = 2; g < 14 ; g=g+2){ // test base to 15 segm and 7 grids
            digitalWrite(VFD_stb, LOW);
            delay(15);
            
            cmd_without_stb((0b11000000) | g);
            Serial.println(d, HEX);
                cmd_without_stb(numbers[d][0] ); // Data to fill table 6*16 = 96 bits
                cmd_without_stb(numbers[d][1] ); // Data to fill table 6*16 = 96 bits
                digitalWrite(VFD_stb, HIGH);
                cmd_with_stb((0b10001000) | 7); //cmd 4
          
                delay(15);
                digitalWrite(VFD_stb, LOW);
                delay(500); //Delay to be possible see the numbers under test!

                cmd_without_stb((0b11000000) | g);
                cmd_without_stb(0b00000000); // Data to fill table 6*16 = 96 bits
                cmd_without_stb(0b00000000); // Data to fill table 6*16 = 96 bits
                digitalWrite(VFD_stb, HIGH);
                delay(15);
                cmd_with_stb((0b10001000) | 7); //cmd 4
                delay(10);
             d++; //Increment by one the position of numbers table!!!
      }
}
void clear_VFD(void){
  /*
  Here I clean all registers 
  Could be done only on the number of grid
  to be more fast. The 12 * 2 bytes = 24 registers
  */
      for (int n=0; n < 21; n++){  // important be 10, if not, bright the half of wells./this on the VFD of 6 grids)
        cmd_with_stb(numberDigits); //       cmd 1 // 7 Grids & 15 Segments
        cmd_with_stb(0b01000000); //       cmd 2 //Normal operation; Set pulse as 1/16
        digitalWrite(VFD_stb, LOW);
        delayMicroseconds(1);
            cmd_without_stb((0b11000000) | n); // cmd 3 //wich define the start address (00H to 15H)
            cmd_without_stb(0b00000000); // Data to fill table of 6 grids * 15 segm = 80 bits on the table
            //
            cmd_with_stb((0b10001000) | 7); //cmd 4
            digitalWrite(VFD_stb, HIGH);
            delayMicroseconds(100);
     }
}
  /******************************************************************/
  /************************** Update Clock **************************/
  /******************************************************************/
void adjustHMS(){
  //This function is only necessáry if you use keys external, not from the Driver.
  //Important is necessary put a pull-up resistor to the VCC(+5VDC) to this pins you use
  //if dont want adjust of the time comment the call of function on the loop
  /* Reset Seconds to 00 Pin number 3 Switch to GND*/
    if((AdjustPins & 0x08) == 0 )
    {
      _delay_ms(200);
      secs=00;
    }
    
    /* Set Minutes when SegCntrl Pin 4 Switch is Pressed*/
    if((AdjustPins & 0x10) == 0 )
    {
      _delay_ms(200);
      if(minutes < 59)
      minutes++;
      else
      minutes = 0;
    }
    /* Set Hours when SegCntrl Pin 5 Switch is Pressed*/
    if((AdjustPins & 0x20) == 0 )
    {
      _delay_ms(200);
      if(hours < 23)
      hours++;
      else
      hours = 0;
    }
}
void readButtons(){
  //Take special attention to the initialize digital pin LED_BUILTIN as an output.
  uint8_t val = 0;       // variable to store the read value
  uint8_t array[9] = {0,0,0,0,0,0,0,0};
  digitalWrite(VFD_stb, LOW);
  delayMicroseconds(2);
  cmd_without_stb(0b01000010); // cmd 2 //Read Keys;Normal operation; Set pulse as 1/16
  pinMode(8, INPUT_PULLUP);  // Important this point! Here I'm changing the direction of the pin to INPUT data.
  delayMicroseconds(2);
  digitalWrite(VFD_clk, LOW);
         for (uint8_t z = 0; z < 3; z++){
                   for (uint8_t h =8; h > 0; h--) {
                    delayMicroseconds(2);
                      digitalWrite(VFD_clk, HIGH);  // Remember wich the read data happen when the clk go from LOW to HIGH! Reverse from write data to out.
                      delayMicroseconds(2);
                      val = digitalRead(8);
                           if (val==1){ 
                            array[h] = 1;
                           }
                           else{ 
                           array[h] = 0;
                           }
                    digitalWrite(VFD_clk, LOW);
                    delayMicroseconds(2);
                   } 
               
              Serial.print(z);  // All the lines of print is only used to debug, comment it, please!
              Serial.print(" - " );
                        
               for (int bits = 8 ; bits > 0; bits--) {
                    Serial.print(array[bits]);
                }
                        
                        if (z==0){
                          if(array[1] == 1){
                           hours=0;
                           minutes=0;
                           secs=0;
                          }
                        }
                          if (z==0){
                          if(array[5] == 1){
                           hours++;
                          }
                          }
                          if (z==0){
                          if(array[6] == 1){
                           hours--;
                          }
                        }
                        if (z==1){
                          if(array[5] == 1){
                           minutes++;
                          }
                        }
                        if (z==0){
                          if(array[2] == 1){
                           minutes--;
                          }
                        }
            Serial.println();
          }  // End of "for" of "z"
      Serial.println();  // This line is only used to debug, please comment it!
 digitalWrite(VFD_stb, HIGH);
 delayMicroseconds(2);
 cmd_with_stb((0b10001000) | 7); //cmd 4
 delayMicroseconds(2);
 pinMode(8, OUTPUT);  // Important this point!  // Important this point! Here I'm changing the direction of the pin to OUTPUT data.
 delay(2); 
}
void updateSetClock(void){
    if (secs >= 60){
      secs = 0;
      minutes++;
    }
    if (minutes >= 60){
      minutes = 0;
      hours++;
    }
    if (hours >= 24){
      hours = 0;
    }
   //*************************************************************
    secU = (secs%10);
    secD = (secs/10);
    updateClock();
    //*************************************************************
    minU = (minutes%10);
    minD = (minutes/10);
    updateClock();
    //**************************************************************
    houU = (hours%10);
    houD = (hours/10);
    updateClock();
    //**************************************************************
}
void updateClock(){
  // This block is very important, it solve the difference 
  // between numbers from grid 1 and grid 2(start 8 or 9)
  uint8_t temp = 0x00;
  digitalWrite(VFD_stb, LOW);
  delayMicroseconds(10);
      cmd_with_stb(numberDigits); // cmd 1 // 5 Grids & 16 numbers
      cmd_with_stb(0b01000000); // cmd 2 //Normal operation; Set pulse as 1/16
        //
        digitalWrite(VFD_stb, LOW);
        delayMicroseconds(10);
        cmd_without_stb((0b11000000) | 0x00); //cmd 3 wich define the start address (00H to 15H)
        //The second byte of array is not used on this case, by this reason is commented.
        cmd_without_stb(0x00);// //----------------------------0 Dummy grid 0
        cmd_without_stb(0x00);// //----------------------------1
        cmd_without_stb(0x00);// //----------------------------2 Dummy grid 1
        cmd_without_stb(0x00);// //----------------------------3
        cmd_without_stb(numbers[houD][0]); //cmd_without_stb(numbers[houD][1]);
        cmd_without_stb(numbers[houU][0]); //cmd_without_stb(numbers[houU][1]);
        //
        cmd_without_stb(numbers[minD][0]);  //cmd_without_stb(numbers[minU][1]);
        cmd_without_stb(numbers[minU][0]); //cmd_without_stb(numbers[minD][1]);
        //
        //Here we start to correct te segment C. The segment C of digit of secD is the
        //segment 0x01 of second Byte which fill memory position on the grid.
        //On this case is the grid #4, address 0x08.
        //cmd_without_stb(numbers[secD][0]); //cmd_without_stb(numbers[secU][1]);
          temp = (numbers[secD][0]);
          temp = temp << 1;  //is necessary make a left shift because segments start at 1 or 2!!!
          cmd_without_stb(temp);
          temp = correctionSeg_C();  //This function return the value of "secU" after be edited!
        cmd_without_stb(temp); //cmd_without_stb(numbers[secD][1]);
        //cmd_without_stb(numbers[secU][0]); //cmd_without_stb(numbers[secU][1]);
        //
      digitalWrite(VFD_stb, HIGH);
      delayMicroseconds(10);
      cmd_with_stb((0b10001000) | 7); //cmd 4
      delay(5);      
}
uint8_t correctionSeg_C(){
  //This function is necessary to correct the segment "C" of first digit of grid #4, address 0x08.
  //I return the value in hexadecimal of segments after be corrected.
  uint8_t tempByteD = 0x00;
  uint8_t tempByteU = 0x00;
                switch (secD){
                case 0: tempByteU =(numbers[secU][0]); 
                        tempByteU = (tempByteU | 0x01); 
                        break;
                case 1: tempByteU =(numbers[secU][0]);
                        break;
                case 2: tempByteU =(numbers[secU][0]); 
                        tempByteU = tempByteU | 0x01;
                        break;
                case 3: tempByteU =(numbers[secU][0]);
                        tempByteU = tempByteU | 0x01;
                        break;
                case 4: tempByteU =(numbers[secU][0]);
                        break;
                case 5: tempByteU =(numbers[secU][0]) ; 
                        tempByteU = tempByteU | 0x01;
                        break;
                case 6: tempByteU =(numbers[secU][0]); 
                        tempByteU = tempByteU | 0x01;
                        break;
                case 7: tempByteU =(numbers[secU][0]);
                        break;
                case 8: tempByteU =(numbers[secU][0]); 
                        tempByteU = tempByteU | 0x01;
                        break;
                case 9: tempByteU =(numbers[secU][0]);
                      break;
                }
    return tempByteU;
}
/******************************************************************/
void setup() {
  // put your setup code here, to run once:
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VFD_in, OUTPUT);
  pinMode(VFD_clk, OUTPUT);
  pinMode(VFD_stb, OUTPUT); // Must be pulsed to LCD fetch data of bus
  Serial.begin(115200);
  seconds = 0x00;
  minutes =0x00;
  hours = 0x00;
  /*CS12  CS11 CS10 DESCRIPTION
  0        0     0  Timer/Counter1 Disabled 
  0        0     1  No Prescaling
  0        1     0  Clock / 8
  0        1     1  Clock / 64
  1        0     0  Clock / 256
  1        0     1  Clock / 1024
  1        1     0  External clock source on T1 pin, Clock on Falling edge
  1        1     1  External clock source on T1 pin, Clock on rising edge
 */
  // initialize timer1 
  cli();           // disable all interrupts
  // initialize timer1 
  //noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;// This initialisations is very important, to have sure the trigger take place!!!
  TCNT1  = 0;
  // Use 62499 to generate a cycle of 1 sex 2 X 0.5 Secs (16MHz / (2*256*(1+62449) = 0.5
  OCR1A = 62498;            // compare match register 16MHz/256/2Hz
  //OCR1A = 500;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= ((1 << CS12) | (0 << CS11) | (0 << CS10));    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
 
  // Note: this counts is done to a Arduino 1 with Atmega 328... Is possible you need adjust
  // a little the value 62499 upper or lower if the clock have a delay or advnce on hours.
    
  //  a=0x33;
  //  b=0x01;
  CLKPR=(0x80);
  //Set PORT
  DDRD = 0xFF;  // IMPORTANT: from pin 0 to 7 is port D, from pin 8 to 13 is port B
  PORTD=0x00;
  DDRB =0xFF;
  PORTB =0x00;
  pt6312_init();
  //test_VFD();
  //clear_VFD();
  //only here I active the enable of interrupts to allow run the test of VFD
  //interrupts();             // enable all interrupts
  sei();
}
void loop() {
  short bitWheel = 0x0001; //This is only used at wheels
  uint8_t wordL = 0x00;  //This is only used at wheels
  uint8_t wordH = 0x00;  //This is only used at wheels
    test_VFD(); //Comment from this line...
    delay(1500); //...
    clear_VFD(); //... 
      while(1){
              if (bitWheel >= 0x2000){ //This is only used at wheels
                bitWheel= 0x0001; //This is only used at wheels
              }
            bitWheel = bitWheel <<=1; //This is only used at wheels
            wordL = bitWheel & 0x00FF; //This is only used at wheels
            wordH = (bitWheel & 0xFF00) >> 8; //This is only used at wheels
        //
      // You can comment untill while cycle to avoid the test running.
      //  test_VFD(); //Comment from this line...
      //  delay(1500); //...
      //  clear_VFD(); //... 
      //  delay(500); //...
      //  test_VFD_grid(); //...
      //  test_VFD_chkGrids(); // until this line to avoid test functions!
        updateSetClock();
        delay(100);
        readButtons();
        delay(100);
        wheels(wordL, wordH);
        delay(100);
      }
}
ISR(TIMER1_COMPA_vect)   {  //This is the interrupt request
      secs++;
}
