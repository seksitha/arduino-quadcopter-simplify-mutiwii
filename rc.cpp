//array getRx(readingRx boolean){
//  if (readingRx == false) {
//    readingRx == true;
//    while (micros() - loop_timer < 650) {
//      if (Serial.available()) {
//        uint8_t val = Serial.read();
//        uint16_t chksum, rxsum;
//        uint8_t i;
//
//        // Look for 0x2040 as start of packet
//        if (ibusIndex == 0 && val != 0x20) {
//          return;
//        }
//        if (ibusIndex == 1 && val != 0x40) {
//          ibusIndex = 0;
//          return;
//        }
//
//        if (ibusIndex < IBUS_BUFFSIZE) {
//          ibus[ibusIndex] = val;
//          ibusIndex++;
//
//        }
//
//        if (ibusIndex == IBUS_BUFFSIZE) {
//          ibusIndex = 0;
//          chksum = 0xFFFF;
//          for (i = 0; i < 30; i++)
//            chksum -= ibus[i];
//          rxsum = ibus[30] + (ibus[31] << 8);
//
//          if (chksum == rxsum) {
//            rcValue[0] = (ibus[3] << 8) + ibus[2];
//            rcValue[1] = (ibus[5] << 8) + ibus[4];
//            rcValue[2] = (ibus[7] << 8) + ibus[6];
//            rcValue[3] = (ibus[9] << 8) + ibus[8];
//            rcValue[4] = (ibus[11] << 8) + ibus[10];
//            rcValue[5] = (ibus[13] << 8) + ibus[12];
//            readingRx = false;
//          }
//        }
//      }
//      if (readingRx == false) {
//        //        PORTD ^= (1 << 7);
//        valueRoll = rcValue[0];     // 2 is the channel
//        valuePitch = rcValue[1];    // 1 is the channel
//        valueThrottle = rcValue[2]; // 3 is the channel
//        valueYaw = rcValue[3];      // 4 is the channel
//        valueAux1 = rcValue[4];     // 5 is the channel
//        valueAux2 = rcValue[5];     // 5 is the channel
//      }
//    }
//
//  }
//}
