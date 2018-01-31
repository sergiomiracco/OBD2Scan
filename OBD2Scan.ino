
#include <Arduino_Due_SD_HSMCI.h>

#include <due_can.h>
#include <iso-tp.h>
#include "obd2_codes.h"
#include "obd2_pid_desc.h"

#ifndef  _VARIANT_MACCHINA_M2_
  #error "You don't seem to be compiling for the M2! Aborting!"
#endif

FileStore FS;

IsoTp isotp0(&Can0);
IsoTp isotp1(&Can1);

#define ECU_STARTID   0x7E0
#define ECU_ENDID     0x7E7
#define REPLY_OFFSET  8 //might be 0x10 or even 0x20 on your car

struct Message_t TxMsg, RxMsg;

void canSetupSpeed(CAN_COMMON *bus, uint32_t defaultSpeed)
{
	bus->enable();
  uint32_t canSpeed = bus->beginAutoSpeed(); //try to figure out the proper speed automatically.
  if (canSpeed > 0)
  {
    SerialUSB.print("Autobaud detected speed is ");
    SerialUSB.println(canSpeed);  
  }
  else
  {
    canSpeed = defaultSpeed;
    SerialUSB.print("Auto speed detect failed. Using ");
    SerialUSB.println(defaultSpeed);
  }  
  bus->begin(canSpeed);
}

void displayBitfield(uint32_t bits)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 7; j > -1; j--)
    {
      if (bits & (1ul << ((i*8) + j))) SerialUSB.write('1');
      else SerialUSB.write('0');
    }
    SerialUSB.write(' ');
  }
  SerialUSB.println();
}
void displaySupportedPidDesc(int n, uint32_t bits)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      if (bits & (1ul << ((i*8) + j)))
      {
        logToSerial(n*32+i*8+j);
        logToSD(n*32+i*8+j);
        //logToSD(PID_DESC[n*32+i*8+j]);
      }
    }
  }
  SerialUSB.println();
}

void queryECU(uint32_t id, IsoTp *iso)
{
  SerialUSB.print("Querying ECU at 0x");
  SerialUSB.println(id, HEX);

  TxMsg.len = 2;
  TxMsg.tx_id = id;
  TxMsg.rx_id = id + REPLY_OFFSET;
  TxMsg.Buffer[0] = OBDII_VEHICLE_INFO;
  TxMsg.Buffer[1] = VI_VIN;  
  iso->send(&TxMsg);
  SerialUSB.print("Reported VIN#: ");
  RxMsg.tx_id = id;
  RxMsg.rx_id = id + REPLY_OFFSET;
  iso->receive(&RxMsg);
  
  if (RxMsg.tp_state == ISOTP_FINISHED)
  {
    for (int i = 2; i < RxMsg.len; i++)  //the first two bytes are the mode and PID so skip those here.
    {
      if (RxMsg.Buffer[i] != 0) SerialUSB.write(RxMsg.Buffer[i]);
    }
    SerialUSB.println();
  }
  else
  {
    SerialUSB.println("ERR!");
  }
  RxMsg.tp_state = ISOTP_IDLE;

  TxMsg.Buffer[0] = OBDII_VEHICLE_INFO;
  TxMsg.Buffer[1] = VI_ECU_NAME;
  iso->send(&TxMsg);
  SerialUSB.print("Reported ECU Name: ");
  iso->receive(&RxMsg);
  if (RxMsg.tp_state == ISOTP_FINISHED)
  {
    for (int i = 2; i < RxMsg.len; i++)
    {
      if (RxMsg.Buffer[i] != 0) SerialUSB.write(RxMsg.Buffer[i]);
    }
    SerialUSB.println();
  }
  else
  {
    SerialUSB.println("ERR!");
  }
  RxMsg.tp_state = ISOTP_IDLE;

  for (int j = 0; j < 5; j++)
  {
    RxMsg.tp_state = ISOTP_IDLE;
    TxMsg.Buffer[0] = OBDII_SHOW_CURRENT;
    TxMsg.Buffer[1] = PID_SUPPORTED1 + (j * 0x20);
    iso->send(&TxMsg);
    SerialUSB.print("Supported PIDs ");
    SerialUSB.print(j * 32 + 1);
    SerialUSB.print("-");
    SerialUSB.print((j * 32) + 32);
    SerialUSB.print(": ");
    iso->receive(&RxMsg);
    if (RxMsg.tp_state == ISOTP_FINISHED)
    {
      uint32_t bits = RxMsg.Buffer[2] + (RxMsg.Buffer[3] * 256) + (RxMsg.Buffer[4] * 65536ul) + (RxMsg.Buffer[5] * 256ul * 65536ul);
      displayBitfield(bits);
      displaySupportedPidDesc(j, bits);
    }
    else
    {
      SerialUSB.println("ERR!");
    }
  }

  SerialUSB.println();
}


void logToSerial(uint8_t n){
  SerialUSB.print(n+1);
  SerialUSB.print(": ");
  SerialUSB.println(PID_DESC[n]);
}

void logToSD(uint8_t n){
  //char write_buffer[sizeof(msg)]; // Creating array of char in length of our string
  //msg.toCharArray(write_buffer,sizeof(msg)); // transform string to array of chars of strings's size
  // Write to file
  String no = String(n,DEC);
  FS.Open("0:","log",true); 
  FS.GoToEnd();
  FS.Write(no.c_str());
  FS.Write(": ");
  FS.Write(PID_DESC[n]);
  FS.Write('\n');  
  FS.Close(); // to save data in file, we must close the file
}

void logToSD(char* msg){
  //char write_buffer[sizeof(msg)]; // Creating array of char in length of our string
  //msg.toCharArray(write_buffer,sizeof(msg)); // transform string to array of chars of strings's size
  // Write to file
  FS.Open("0:","log",true); 
  FS.GoToEnd(); 
  FS.Write(msg);
  FS.Write('\n');  
  FS.Close(); // to save data in file, we must close the file
}

void setup()
{
  delay(4000);

  // Check if there is card inserted
  SD.Init(); // Initialization of HSCMI protocol and SD socket switch GPIO (to adjust pin number go to library source file - check Getting Started Guide)
  FS.Init(); // Initialization of FileStore object for file manipulation

  char message_init[] = "OBDII Scanner for M2"; 

  /**init SD Card log**/
  FS.CreateNew("0:","log"); // Create new file, if alredy exists it will be overwritten
  //FS.GoToEnd(); // Do not need when creating file because new file is opened and position 0
  FS.Write(message_init); // writing message
  FS.Write('\n');
  FS.Close(); //close file to store 

  /**init Serial log**/
  SerialUSB.begin(1000000);
  SerialUSB.println(message_init);
  SerialUSB.println();

  SerialUSB.print("CAN0:");
  canSetupSpeed(&Can0, 500000);
  SerialUSB.print("CAN1:");
  canSetupSpeed(&Can1, 250000);
  
  for (int filter = 0; filter < 3; filter++) {
      Can0.setRXFilter(filter, 0, 0, true);
      Can1.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
      Can0.setRXFilter(filter, 0, 0, false);
      Can1.setRXFilter(filter, 0, 0, false);
  }  

  TxMsg.Buffer=(uint8_t *)calloc(MAX_MSGBUF,sizeof(uint8_t));
  RxMsg.Buffer=(uint8_t *)calloc(MAX_MSGBUF,sizeof(uint8_t));

  /*
  The OBDII standard says that ECUs should respond to ids in the range 0x7E0 to 0x7E7. They should
  respond 0x08 higher than their ID so 0x7E0 responds with ID 0x7E8. Some cars don't follow this standard
  very closely and thus use addresses outside this range and return offsets other than 0x8. 0x10 is common too.
  Modify REPLY_OFFSET to change the offset. Modify ECU_STARTID and ECU_ENDID to change the range.
  */

  SerialUSB.println("----==== CAN0 ECUS ====----");
  uint32_t ecuID;
  for (ecuID = ECU_STARTID; ecuID <= ECU_ENDID; ecuID++)
  {
    queryECU(ecuID, &isotp0);
  }

  SerialUSB.println("----==== CAN1 ECUS ====----");
  for (ecuID = ECU_STARTID; ecuID <= ECU_ENDID; ecuID++)
  {
    queryECU(ecuID, &isotp1);
  }

}

void loop()
{ 
}

