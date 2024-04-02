#include "Plane.h"

uint8_t Plane::CRC8Sum(uint8_t* data, size_t length) {
  uint8_t crc = 0x00;
  for(size_t j = 0; j < length; j++) {
      crc ^= data[j];
      for(int i = 0; i < 8; i++) {
          if((crc & 0x80) != 0)
              crc = (uint8_t)((crc << 1) ^ 0x07);
          else
              crc <<= 1;
      }
  }
  return crc;
}

float Plane::GetFloatValue_(char fir, char sec, char thr, char fou) {
	float f;
	char b[] = { fir, sec, thr, fou };
	memcpy(&f, &b, sizeof(f));
	return f;
}

void Plane::setup_uart(AP_HAL::UARTDriver *uart, const char *name) {
	    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(115200);
	//uart->begin(57600);
}

uint8_t* Plane::SendMessToXplane(float Lailn1, float Railn1, float elev1, float elev1_2) {
	uint8_t var_Lailn1[4], var_Railn1[4], var_elev1[4], var_elev1_2[4];

	union CONVERTFLOAT
	{
		float ConvertValue;
		uint32_t Data32Conv;
		uint8_t DataFloat[4];
	}ConvertFloat;

	ConvertFloat.ConvertValue = Lailn1;
	memcpy(var_Lailn1, ConvertFloat.DataFloat, 4);

	ConvertFloat.ConvertValue = Railn1;
	memcpy(var_Railn1, ConvertFloat.DataFloat, 4);

	ConvertFloat.ConvertValue = elev1;
	memcpy(var_elev1, ConvertFloat.DataFloat, 4);

  ConvertFloat.ConvertValue = elev1_2;
  memcpy(var_elev1_2, ConvertFloat.DataFloat, 4);

	uint8_t buffer[18];

	buffer[0]  = 0x58;
    buffer[1]  = 0x50;

	buffer[2]  = static_cast<uint8_t>(var_Lailn1[0]);
	buffer[3]  = static_cast<uint8_t>(var_Lailn1[1]);
	buffer[4]  = static_cast<uint8_t>(var_Lailn1[2]);
	buffer[5]  = static_cast<uint8_t>(var_Lailn1[3]);

	buffer[6]  = static_cast<uint8_t>(var_Railn1[0]);
	buffer[7]  = static_cast<uint8_t>(var_Railn1[1]);
	buffer[8]  = static_cast<uint8_t>(var_Railn1[2]);
	buffer[9]  = static_cast<uint8_t>(var_Railn1[3]);

	buffer[10] = static_cast<uint8_t>(var_elev1[0]);
	buffer[11] = static_cast<uint8_t>(var_elev1[1]);
	buffer[12] = static_cast<uint8_t>(var_elev1[2]);
	buffer[13] = static_cast<uint8_t>(var_elev1[3]);

    buffer[14] = static_cast<uint8_t>(var_elev1_2[0]);
    buffer[15] = static_cast<uint8_t>(var_elev1_2[1]);
    buffer[16] = static_cast<uint8_t>(var_elev1_2[2]);
    buffer[17] = static_cast<uint8_t>(var_elev1_2[3]);

	uint8_t sum = CRC8Sum(buffer, 18);

	static uint8_t Mess[19];
	Mess[0]  = 0x58;
    Mess[1]  = 0x50;
	Mess[2]  = var_Lailn1[0];
	Mess[3]  = var_Lailn1[1];
	Mess[4]  = var_Lailn1[2];
	Mess[5]  = var_Lailn1[3];
	Mess[6]  = var_Railn1[0];
	Mess[7]  = var_Railn1[1];
	Mess[8]  = var_Railn1[2];
	Mess[9]  = var_Railn1[3];
	Mess[10] = var_elev1[0];
	Mess[11] = var_elev1[1];
	Mess[12] = var_elev1[2];
	Mess[13] = var_elev1[3];
    Mess[14] = var_elev1_2[0];
    Mess[15] = var_elev1_2[1];
    Mess[16] = var_elev1_2[2];
    Mess[17] = var_elev1_2[3];
	Mess[18] = static_cast<uint8_t>(sum);

	return Mess;
}

uint8_t* Plane::SendMessToXplane2(float elev1, float Ailn1, float rudr1, float throl1_from_AP,
								  float throl2_from_AP, float start_stop_left_ENG, float start_stop_right_ENG,
								  float brake_parking_from_AP, float gear_from_AP) {
	
	uint8_t var_elev1[4], var_Ailn1[4], var_rudr1[4], var_throl1_from_AP[4],
			var_throl2_from_AP[4], var_start_stop_left_ENG[4], var_start_stop_right_ENG[4],
			var_brake_parking_from_AP[4], var_gear_from_AP[4];

	union CONVERTFLOAT
	{
		float ConvertValue;
		uint32_t Data32Conv;
		uint8_t DataFloat[4];
	}ConvertFloat;

	ConvertFloat.ConvertValue = elev1;
	memcpy(var_elev1, ConvertFloat.DataFloat, 4);

	ConvertFloat.ConvertValue = Ailn1;
	memcpy(var_Ailn1, ConvertFloat.DataFloat, 4);

	ConvertFloat.ConvertValue = rudr1;
	memcpy(var_rudr1, ConvertFloat.DataFloat, 4);

	ConvertFloat.ConvertValue = throl1_from_AP;
	memcpy(var_throl1_from_AP, ConvertFloat.DataFloat, 4);
	
	ConvertFloat.ConvertValue = throl2_from_AP;
	memcpy(var_throl2_from_AP, ConvertFloat.DataFloat, 4);
	
	ConvertFloat.ConvertValue = start_stop_left_ENG;
	memcpy(var_start_stop_left_ENG, ConvertFloat.DataFloat, 4);
	
	ConvertFloat.ConvertValue = start_stop_right_ENG;
	memcpy(var_start_stop_right_ENG, ConvertFloat.DataFloat, 4);

	ConvertFloat.ConvertValue = brake_parking_from_AP;
	memcpy(var_brake_parking_from_AP, ConvertFloat.DataFloat, 4);

	ConvertFloat.ConvertValue = gear_from_AP;
	memcpy(var_gear_from_AP, ConvertFloat.DataFloat, 4);

	uint8_t buffer[18];

	buffer[0]  = 0xAA;
    buffer[1]  = 0x55;
 	buffer[2]  = 0x00;
	buffer[3]  = 0x00;
	
	buffer[4]  = static_cast<uint8_t>(var_elev1[0]);
	buffer[5]  = static_cast<uint8_t>(var_elev1[1]);
	buffer[6]  = static_cast<uint8_t>(var_elev1[2]);
	buffer[7]  = static_cast<uint8_t>(var_elev1[3]);

	buffer[8]  = static_cast<uint8_t>(var_Ailn1[0]);
	buffer[9]  = static_cast<uint8_t>(var_Ailn1[1]);
	buffer[10]  = static_cast<uint8_t>(var_Ailn1[2]);
	buffer[11]  = static_cast<uint8_t>(var_Ailn1[3]);

	buffer[12] = static_cast<uint8_t>(var_rudr1[0]);
	buffer[13] = static_cast<uint8_t>(var_rudr1[1]);
	buffer[14] = static_cast<uint8_t>(var_rudr1[2]);
	buffer[15] = static_cast<uint8_t>(var_rudr1[3]);

    buffer[16] = static_cast<uint8_t>(var_throl1_from_AP[0]);
    buffer[17] = static_cast<uint8_t>(var_throl1_from_AP[1]);
    buffer[18] = static_cast<uint8_t>(var_throl1_from_AP[2]);
    buffer[19] = static_cast<uint8_t>(var_throl1_from_AP[3]);

    buffer[20] = static_cast<uint8_t>(var_throl2_from_AP[0]);
    buffer[21] = static_cast<uint8_t>(var_throl2_from_AP[1]);
    buffer[22] = static_cast<uint8_t>(var_throl2_from_AP[2]);
    buffer[23] = static_cast<uint8_t>(var_throl2_from_AP[3]);

	buffer[24] = static_cast<uint8_t>(var_start_stop_left_ENG[0]);
    buffer[25] = static_cast<uint8_t>(var_start_stop_left_ENG[1]);
    buffer[26] = static_cast<uint8_t>(var_start_stop_left_ENG[2]);
    buffer[27] = static_cast<uint8_t>(var_start_stop_left_ENG[3]);

	buffer[28] = static_cast<uint8_t>(var_start_stop_right_ENG[0]);
    buffer[29] = static_cast<uint8_t>(var_start_stop_right_ENG[1]);
    buffer[30] = static_cast<uint8_t>(var_start_stop_right_ENG[2]);
    buffer[31] = static_cast<uint8_t>(var_start_stop_right_ENG[3]);

	buffer[32] = static_cast<uint8_t>(var_gear_from_AP[0]);
    buffer[33] = static_cast<uint8_t>(var_gear_from_AP[1]);
    buffer[34] = static_cast<uint8_t>(var_gear_from_AP[2]);
    buffer[35] = static_cast<uint8_t>(var_gear_from_AP[3]);
	
	buffer[36] = static_cast<uint8_t>(var_brake_parking_from_AP[0]);
    buffer[37] = static_cast<uint8_t>(var_brake_parking_from_AP[1]);
    buffer[38] = static_cast<uint8_t>(var_brake_parking_from_AP[2]);
    buffer[39] = static_cast<uint8_t>(var_brake_parking_from_AP[3]);
	
	buffer[40] = static_cast<uint8_t>(0x00);
    buffer[41] = static_cast<uint8_t>(0x00);
    buffer[42] = static_cast<uint8_t>(0x00);
    buffer[43] = static_cast<uint8_t>(0x00);
	
	buffer[44] = static_cast<uint8_t>(0x00);
    buffer[45] = static_cast<uint8_t>(0x00);
    buffer[46] = static_cast<uint8_t>(0x00);
    buffer[47] = static_cast<uint8_t>(0x00);
	
	buffer[48] = static_cast<uint8_t>(0x00);
    buffer[49] = static_cast<uint8_t>(0x00);
    buffer[50] = static_cast<uint8_t>(0x00);
    buffer[51] = static_cast<uint8_t>(0x00);
	
	uint8_t sum = CRC8Sum(buffer, 18);

	static uint8_t Mess[54];
	Mess[0]  = 0xAA;
    Mess[1]  = 0x55;
	Mess[2]  = 0x00;
	Mess[3]  = 0x00;
	
	Mess[4]  = var_elev1[0];
	Mess[5]  = var_elev1[1];
	Mess[6]  = var_elev1[2];
	Mess[7]  = var_elev1[3];
	
	Mess[8]  = var_Ailn1[0];
	Mess[9]  = var_Ailn1[1];
	Mess[10] = var_Ailn1[2];
	Mess[11] = var_Ailn1[3];
	
	Mess[12] = var_rudr1[0];
	Mess[13] = var_rudr1[1];
    Mess[14] = var_rudr1[2];
    Mess[15] = var_rudr1[3];
    
	Mess[16] = var_throl1_from_AP[0];
    Mess[17] = var_throl1_from_AP[1];
	Mess[18] = var_throl1_from_AP[2];
	Mess[19] = var_throl1_from_AP[3];

	Mess[20] = var_throl2_from_AP[0];
	Mess[21] = var_throl2_from_AP[1];
	Mess[22] = var_throl2_from_AP[2];
	Mess[23] = var_throl2_from_AP[3];

	Mess[24] = var_start_stop_left_ENG[0]; 
	Mess[25] = var_start_stop_left_ENG[1];
	Mess[26] = var_start_stop_left_ENG[2];
	Mess[27] = var_start_stop_left_ENG[3];

	Mess[28] = var_start_stop_right_ENG[0]; 
	Mess[29] = var_start_stop_right_ENG[1];
	Mess[30] = var_start_stop_right_ENG[2];
	Mess[31] = var_start_stop_right_ENG[3];

	Mess[32] = var_gear_from_AP[0]; 
	Mess[33] = var_gear_from_AP[1];
	Mess[34] = var_gear_from_AP[2];
	Mess[35] = var_gear_from_AP[3];

	Mess[36] = var_brake_parking_from_AP[0]; 
	Mess[37] = var_brake_parking_from_AP[1];
	Mess[38] = var_brake_parking_from_AP[2];
	Mess[39] = var_brake_parking_from_AP[3];

	Mess[40] = 0x00; 
	Mess[41] = 0x00;
	Mess[42] = 0x00;
	Mess[43] = 0x00;

	Mess[44] = 0x00; 
	Mess[45] = 0x00;
	Mess[46] = 0x00;
	Mess[47] = 0x00;

	Mess[48] = 0x00; 
	Mess[49] = 0x00;
	Mess[50] = 0x00;
	Mess[51] = 0x00;

	Mess[52] = 0x00; 
	Mess[53] = 0x00;

	return Mess;
}


void Plane::ReadDataFromXplane() {
  if(senddata_t2 == true)
	{
		hal.scheduler->delay(1000);
		setup_uart(hal.uartC, "uartC"); // telemetry 1
		setup_uart(hal.uartD, "uartD");  // telemetry 2
		senddata_t2 = false;
		//hal.scheduler->delay(100);
	}

	bRec = 0;
    
    const int MESSAGE_SIZE = 63;
    const uint8_t START_BYTE = 0x58;
    const uint8_t SECOND_BYTE = 0x50;
    
    static uint8_t buffer[MESSAGE_SIZE * 2];
    static int bytesInBuffer = 0;
    uint8_t processedMessage[MESSAGE_SIZE];
    
	while (hal.uartC->available() && bRec <= 512)
	{
		// simbol = hal.uartC->read();
		// messReceive[bRec] = simbol;
		// bRec = bRec + 1;
        buffer[bytesInBuffer++] = hal.uartC->read();
        if(bytesInBuffer >= MESSAGE_SIZE * 2){
            for(int i = 0; i <= bytesInBuffer - MESSAGE_SIZE; i++){
                if(buffer[i] == START_BYTE && buffer [i + 1] == SECOND_BYTE){
                    memcpy(processedMessage, &buffer[i], MESSAGE_SIZE);
                    
                    memmove(buffer, &buffer[i + MESSAGE_SIZE], bytesInBuffer - (i + MESSAGE_SIZE));
                
                    bytesInBuffer -= (i + MESSAGE_SIZE);
                    break;
                }
            }
        }
	}

	// char buffer[bRec + 1];
	// memcpy(buffer, messReceive, bRec);
	// buffer[bRec] = '\0';

	// hal.uartD->printf("   bRec = %d\n", bRec);
    // hal.uartD->printf("buffer = %c\n", buffer[62]);
    // hal.uartD->printf("CRC8 = %c\n", CRC8Sum((uint8_t*)buffer, 62));


    
	if ((unsigned char)processedMessage[0] == 0x58 && (unsigned char)processedMessage[1] == 0x50) {

		if ( CRC8Sum((uint8_t*)processedMessage, 62) == (uint8_t)processedMessage[62])
		{
            trueSpd_Xplane    = GetFloatValue_(buffer[2], buffer[3], buffer[4], buffer[5]);
            vvi_XPlane        = GetFloatValue_(buffer[6], buffer[7], buffer[8], buffer[9]);
            w_y_XPlane 		    = GetFloatValue_(buffer[10], buffer[11], buffer[12], buffer[13]);
            w_x_Xplane 		    = GetFloatValue_(buffer[14], buffer[15], buffer[16], buffer[17]);
            w_z_Xplane 		    = GetFloatValue_(buffer[18], buffer[19], buffer[20], buffer[21]);
            pitch_Xplane 		  = GetFloatValue_(buffer[22], buffer[23], buffer[24], buffer[25]);
            roll_Xplane 		  = GetFloatValue_(buffer[26], buffer[27], buffer[28], buffer[29]);
            hding_true_Xplane = GetFloatValue_(buffer[30], buffer[31], buffer[32], buffer[33]);
            hding_mag_Xplane 	= GetFloatValue_(buffer[34], buffer[35], buffer[36], buffer[37]);
            alpha_Xplane 		  = GetFloatValue_(buffer[38], buffer[39], buffer[40], buffer[41]);
            beta_Xplane 	  	= GetFloatValue_(buffer[42], buffer[43], buffer[44], buffer[45]);
            lat_Xplane 		    = GetFloatValue_(buffer[46], buffer[47], buffer[48], buffer[49]);
            lon_Xplane 		    = GetFloatValue_(buffer[50], buffer[51], buffer[52], buffer[53]);
            alt_Xplane        = GetFloatValue_(buffer[54], buffer[55], buffer[56], buffer[57]);
            GroundSpd_Xplane = GetFloatValue_(buffer[58], buffer[59], buffer[60], buffer[61]);
			// hal.uartC->write((const uint8_t*)SendMess_A1(target_point_lat, target_point_lng, target_point_alt), 14);
      //
      //
			// hal.uartD->printf("-----------------------------------------\n");
			// hal.uartD->printf("pitchController = %lf\n", pitch_Xplane);
			// hal.uartD->printf("roll = %lf\n", roll_Xplane);
			// hal.uartD->printf("yaw = %lf\n", hding_true_Xplane);
		}
	}
  /*******************************************************/
  /******************* РУЛИ ******************************/
  /*******************************************************/
  // Lailn1_Xplane  = ahrs.roll_sensor*0.01f;  // left aileron
  // Railn1_Xplane  = ahrs.pitch_sensor*0.01f; // rigth aileron
  // elev1_Xplane   = ahrs.yaw_sensor*0.01f;   // elevator
  // elev1_2_Xplane = ahrs.yaw_sensor*0.01f;   // elevator
  /*******************************************************/
  /*******************************************************/
  /*******************************************************/

  hal.uartD->write((const uint8_t*)SendMessToXplane(Lailn1_Xplane, Railn1_Xplane, elev1_Xplane, elev1_2_Xplane), 19);

}
