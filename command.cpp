// char buf[80]; //???

// int readline(int readch, char *buffer, int len) { //  char *buffer???
    
//     static int pos = 0; // static every time it call it will not reset value;
//     int rpos; // value get reset

//     if (readch > 0) {
//         switch (readch) {
//             case '\r': // return nothing and do nothing if \r
//                 break;
//             case '\n': 
//                 rpos = pos;
//                 pos = 0;  // Reset position index ready for next time
//                 return rpos; // Return index > 0 so that in the loop it will print the buf-string
//             default: // if it is not end of line 
//                 if (pos < len-1) {
//                     buffer[pos++] = readch; // adding string one by one of charactor
//                     buffer[pos] = 0; // ???
//                 }
//         }
//     }

//     return 0; // when it is not a line break return 0 so that the loop is not print anything.
// }

// void setup() {
//     Serial.begin(115200);
// }

// void loop() {
//     if (readline(Serial.read(), buf, 80) > 0) {
//         Serial.print("You entered: >");
//         Serial.print(buf);
//         Serial.println("<");
//     }
// }

  looptimer = micros();
    while (Serial.available()) {
      val = Serial.read();
      
      if(ind==0 && val !=32){
        return;
      }
      
      if(ind==1 && val !=64){
        ind = 0;
        return;
       
      }
      
      if(ind < 6){
        myar[ind]=val;
        ind++;
      }
      
      
    }
  
  if (ind == 6){
    //Serial.println(myar[0]);	
  	//Serial.println(myar[1]);
  	//Serial.println(myar[2]+(myar[3]<<8));
  	//Serial.println(myar[4]+(myar[5]<<8));
    (PORTB ^= (1 << 1));
   	ind=0;
  }
  //Serial.println(micros()-looptimer);
  delay(30);