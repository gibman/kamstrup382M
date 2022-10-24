#include "esphome.h"
unsigned const long sampletime_ms = 15000;
static const char* TAG = "Kamstrup382M"; // Logging tag
static const word  kamnums[]    = { 0x0001,0x0002,0x000d,0x000e,0x041e,0x041f,0x0420,0x0434,0x0435,0x0436,0x0438,0x0439,0x043a,0x0056,0x0057,0x0058,0x007A,0x0059,0x005B,0x005C,0x004A,0x004B,0x03ff,0x0438,0x0439,0x043a,0x0400,0x0540,0x0541,0x0542 };
static const char* kamstrings[] = {"Energy in","Energy out","Energy in hi-res","Energy out hi-res","Voltage p1","Voltage p2","Voltage p3","Current p1","Current p2","Current p3","Power p1","Power p2","Power p3","Current flow temperature","Current return flow temperature","Current temperature T3","Current temperature T4","Current temperature difference","Pressure in flow","Pressure in return flow","Current flow in flow","Current flow in return flow","Power In","Power p1 In","Power p2 In","Power p3 In","Power Out","Power p1 Out","Power p2 Out","Power p3 Out" };  
#define KAMTIMEOUT 300  // Kamstrup timeout after transmit

// Units
static const char*  units[65] = {"","Wh","kWh","MWh","GWh","j","kj","Mj",
        "Gj","Cal","kCal","Mcal","Gcal","varh","kvarh","Mvarh","Gvarh",
        "VAh","kVAh","MVAh","GVAh","kW","kW","MW","GW","kvar","kvar","Mvar",
        "Gvar","VA","kVA","MVA","GVA","V","A","kV","kA","C","K","l","m3",
        "l/h","m3/h","m3xC","ton","ton/h","h","hh:mm:ss","yy:mm:dd","yyyy:mm:dd",
        "mm:dd","","bar","RTC","ASCII","m3 x 10","ton x 10","GJ x 10","minutes","Bitfield",
        "s","ms","days","RTC-Q","Datetime"};


class Kamstrup382M : public PollingComponent, public UARTDevice {

  private:    
    // crc_1021 - calculate crc16
    long crc_1021(byte const *inmsg, unsigned int len){
      long creg = 0x0000;
      for(unsigned int i = 0; i < len; i++) {
        int mask = 0x80;
        while(mask > 0) {
          creg <<= 1;
          if (inmsg[i] & mask){
            creg |= 1;
          }
          mask>>=1;
          if (creg & 0x10000) {
            creg &= 0xffff;
            creg ^= 0x1021;
          }
        }
      }
      return creg;
    }
         
    // kamReceive - receive bytes from Kamstrup meter
    unsigned short kamReceive(byte recvmsg[]) {

      byte rxdata[50];  // buffer to hold received data
      unsigned long rxindex = 0;
      unsigned long starttime = millis();
      
      flush();  // flush serial buffer - might contain noise

      byte r;
      
      // loop until EOL received or timeout
      while(r != 0x0d) {
        
        // handle rx timeout
        if(millis()-starttime > KAMTIMEOUT) {
          ESP_LOGD(TAG, "Timed out listening for data");
          return 0;
        }

        // handle incoming data
        if (available()) {

          // receive byte                   
          read_byte(&r);
          if(r != 0x40) {  // don't append if we see the start marker
            // append data
            rxdata[rxindex] = r;
            rxindex++; 
          }
        }
      }

      // remove escape markers from received data
      unsigned short j = 0;
      for (unsigned short i = 0; i < rxindex -1; i++) {
        if (rxdata[i] == 0x1b) {
          byte v = rxdata[i+1] ^ 0xff;
          if (v != 0x06 and v != 0x0d and v != 0x1b and v != 0x40 and v != 0x80){
            ESP_LOGD(TAG, "Missing escape '%b'", v);
          }
          recvmsg[j] = v;
          i++; // skip
        } else {
          recvmsg[j] = rxdata[i];
        }
        j++;
      }
      
      // check CRC
      if (crc_1021(recvmsg,j)) {
        ESP_LOGD(TAG, "CRC error: '%b'", recvmsg[j]);
        return 0;
      }
      
      return j;      
    }

  
    // kamReadReg - read a Kamstrup register
    float kamReadReg(unsigned short kreg, bool &hasReply) {

      hasReply = false;
      byte recvmsg[30];  // buffer of bytes to hold the received data
      float rval = 0;    // this will hold the final value

      // prepare message to send and send it
      byte sendmsg[] = { 0x3f, 0x10, 0x01, (kamnums[kreg] >> 8), (kamnums[kreg] & 0xff) };
      kamSend(sendmsg, 5);

      // listen if we get an answer
      unsigned short rxnum = kamReceive(recvmsg);

      // check if number of received bytes > 0 
      if(rxnum != 0) {          
        // decode the received message
        rval = kamDecode(kreg, recvmsg);
        hasReply = true;                    
        ESP_LOGD(TAG, "%s: %f %s", kamstrings[kreg], rval, units[recvmsg[4]]);            		  
      }
      return rval;
    }
  
    // kamDecode - decodes received data
    float kamDecode(unsigned short const kreg, byte const *msg) {
      // skip if message is not valid
      if (msg[0] != 0x3f or msg[1] != 0x10) {
        return false;
      }
      if (msg[2] != (kamnums[kreg] >> 8) or msg[3] != (kamnums[kreg] & 0xff)) {
        return false;
      }
        
      // decode the mantissa
      long x = 0;
      for (int i = 0; i < msg[5]; i++) {
        x <<= 8;
        x |= msg[i + 7];
      }
      
      // decode the exponent
      int i = msg[6] & 0x3f;
      if (msg[6] & 0x40) {
        i = -i;
      };
      float ifl = pow(10,i);
      if (msg[6] & 0x80) {
        ifl = -ifl;
      }

      // return final value
      return (float)(x * ifl);
    }
  
    // kamSend - send data to Kamstrup meter
    void kamSend(byte const *msg, int msgsize) {

      // append checksum bytes to message
      byte newmsg[msgsize+2];
      for (int i = 0; i < msgsize; i++) { newmsg[i] = msg[i]; }
      newmsg[msgsize++] = 0x00;
      newmsg[msgsize++] = 0x00;
      int c = crc_1021(newmsg, msgsize);
      newmsg[msgsize-2] = (c >> 8);
      newmsg[msgsize-1] = c & 0xff;

      // build final transmit message - escape various bytes
      byte txmsg[20] = { 0x80 };   // prefix
      int txsize = 1;
      for (int i = 0; i < msgsize; i++) {
        if (newmsg[i] == 0x06 or newmsg[i] == 0x0d or newmsg[i] == 0x1b or newmsg[i] == 0x40 or newmsg[i] == 0x80) {
          txmsg[txsize++] = 0x1b;
          txmsg[txsize++] = newmsg[i] ^ 0xff;
        } else {
          txmsg[txsize++] = newmsg[i];
        }
      }
      txmsg[txsize++] = 0x0d;  // EOF

      // send to serial interface
      for (int x = 0; x < txsize; x++) {
        write_byte(txmsg[x]);
       
      }

    }
  
  public:  
    // For each of the values we wish to export, we define a sensor
    Sensor *energy_in = new Sensor();
    Sensor *energy_out = new Sensor();
    Sensor *energy_in_hi_res = new Sensor();
    Sensor *energy_out_hi_res = new Sensor();
    Sensor *voltage_p1 = new Sensor();
    Sensor *voltage_p2 = new Sensor();
    Sensor *voltage_p3 = new Sensor();
    Sensor *current_p1 = new Sensor();
    Sensor *current_p2 = new Sensor();
    Sensor *current_p3 = new Sensor();
    Sensor *power_p1 = new Sensor();
    Sensor *power_p2 = new Sensor();
    Sensor *power_p3 = new Sensor();
    Sensor *current_flow_temperature = new Sensor();
    Sensor *current_return_flow_temperature = new Sensor();
    Sensor *current_temperature_t3 = new Sensor();
    Sensor *current_temperature_t4 = new Sensor();
    Sensor *current_temperature_difference = new Sensor();
    Sensor *pressure_in_flow = new Sensor();
    Sensor *pressure_in_return_flow = new Sensor();
    Sensor *current_flow_in_flow = new Sensor();
    Sensor *current_flow_in_return_flow = new Sensor();
    Sensor *power_in = new Sensor();
    Sensor *power_p1_in = new Sensor();
    Sensor *power_p2_in = new Sensor();
    Sensor *power_p3_in = new Sensor();
    Sensor *power_out = new Sensor();
    Sensor *power_p1_out = new Sensor();
    Sensor *power_p2_out = new Sensor();
    Sensor *power_p3_out = new Sensor();
   
    Kamstrup382M(UARTComponent *parent) : PollingComponent(sampletime_ms), UARTDevice(parent) {};
   
    void update() override
    {
     
      ESP_LOGD(TAG, "reading kamstrup meter");
      for (int kreg = 0; kreg < sizeof kamnums/sizeof kamnums[0]; kreg++) {
        bool hasReply;
        float val = kamReadReg(kreg, hasReply);
        
        if (!hasReply)
        {            
          continue;
        }
        
        switch(kreg) {
          case 0:
            if (energy_in->state != val) {
              energy_in->publish_state(val);
            }
            break;
            
          case 1:
            if (energy_out->state != val) {
              energy_out->publish_state(val);
            }
            break;
            
          case 2:
            if (energy_in_hi_res->state != val) {
              energy_in_hi_res->publish_state(val);
            }
            break;
        
          case 3:
            if (energy_out_hi_res->state != val) {
              energy_out_hi_res->publish_state(val);
            }
            break;
          case 4:
            if (voltage_p1->state != val) {
              voltage_p1->publish_state(val);
            }
            break;
            
          case 5:
            if (voltage_p2->state != val) {
              voltage_p2->publish_state(val);
            }
            break;
            
          case 6:
            if (voltage_p3->state != val) {
              voltage_p3->publish_state(val);
            }
            break;
            
          case 7:
            if (current_p1->state != val) {
              current_p1->publish_state(val);
            }
            break;
            
          case 8:
            if (current_p2->state != val) {
              current_p2->publish_state(val);
            }
            break;
            
          case 9:
            if (current_p3->state != val) {
              current_p3->publish_state(val);
            }
            break;
            
          case 10:
            if (power_p1->state != val) {
              power_p1->publish_state(val);
            }
            break;
            
          case 11:
            if (power_p2->state != val) {
              power_p2->publish_state(val);
            }
            break;
            
          case 12:
            if (power_p3->state != val) {
              power_p3->publish_state(val);
            }
            break;
            
          case 13:
            if (current_flow_temperature->state != val) {
              current_flow_temperature->publish_state(val);
            }
            break;
            
          case 14:
            if (current_return_flow_temperature->state != val) {
              current_return_flow_temperature->publish_state(val);
            }
            break;
            
          case 15:
            if (current_temperature_t3->state != val) {
              current_temperature_t3->publish_state(val);
            }
            break;
            
          case 16:
            if (current_temperature_t4->state != val) {
              current_temperature_t4->publish_state(val);
            }
            break;
            
          case 17:
            if (current_temperature_difference->state != val) {
              current_temperature_difference->publish_state(val);
            }
            break;
            
          case 18:
            if (pressure_in_flow->state != val) {
              pressure_in_flow->publish_state(val);
            }
            break;
            
          case 19:
            if (pressure_in_return_flow->state != val) {
              pressure_in_return_flow->publish_state(val);
            }
            break;
            
          case 20:
            if (current_flow_in_flow->state != val) {
              current_flow_in_flow->publish_state(val);
            }
            break;
            
          case 21:
            if (current_flow_in_return_flow->state != val) {
              current_flow_in_return_flow->publish_state(val);
            }
            break;
            
          case 22:
            if (power_in->state != val) {
              power_in->publish_state(val);
            }
            break;
            
          case 23:
            if (power_p1_in->state != val) {
              power_p1_in->publish_state(val);
            }
            break;
            
          case 24:
            if (power_p2_in->state != val) {
              power_p2_in->publish_state(val);
            }
            break;
            
          case 25:
            if (power_p3_in->state != val) {
              power_p3_in->publish_state(val);
            }
            break;
            
          case 26:
            if (power_out->state != val) {
              power_out->publish_state(val);
            }
            break;
            
          case 27:
            if (power_p1_out->state != val) {
              power_p1_out->publish_state(val);
            }
            break;
            
          case 28:
            if (power_p2_out->state != val) {
              power_p2_out->publish_state(val);
            }
            break;
            
          case 29:
            if (power_p3_out->state != val) {
              power_p3_out->publish_state(val);
            }
            break;
        }//end of switch case...
      }
    
    }
     
    void setup() override {
      // nothing to do here                
    }
  
};
  

