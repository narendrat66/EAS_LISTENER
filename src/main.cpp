/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos.h"
#include "mgos_app.h"
#include "mgos_mqtt.h"
#include "mgos_gpio.h"
#include "mgos_arduino_lora.h"
#include "mgos_arduino.h"
/**** Delta Things IoT Cloud standard topics ****/
#define  TELEMETRY_TOPIC "v1/devices/me/telemetry"

#define START_BYTE 0x02
#define END_BYTE 0x03
#define LORA_BUF_SIZE 30
#define ACK_BUF_SIZE 6

#define MODBUS_DATA_LENGTH 10
#define MODBUS_RX_DATA_LN 8
#define MODBUS_TX_DATA_LN 30
#define LORA_DATA_LENGTH 23

#define MODBUS_SLAVE_ID 0x01
#define MODBUS_FUNC_COD 0x03

#define MODBUS_SLAVE_ID_INDEX 0
#define MODBUS_FUNC_COD_INDEX 1
#define MODBUS_TXN_SIZE_INDEX 2
#define MODBUS_DATA_STT_INDEX 3

uint8_t Modbus_Rx_Pkt[MODBUS_RX_DATA_LN] = {0};
static uint8_t Modbus_Tx_Pkt[MODBUS_TX_DATA_LN] = {0x01,0x03,0x0E,0x00,0x65,0x00,0x01,0x00,0x2D,0x06,0xB0,0x0D,0x80,0x1E,0x83,0x04,0x57,0x22,0xC1};

LoRaClass* lora;

/*****   RAW Packet Indexes    ************/
#define START_BYTE_INDEX 0
#define STOP_BYTE_INDEX  22

/*****   Extracted Packet Indexes    ******/
#define NODE_MODE_INDEX  0
#define DEVICE_ID_INDEX  0
#define NODE_ID_INDEX    9
#define BUTTON_ST_INDEX  10
#define BATTERY_VL_INDEX 10
#define LATITUDE_INDEX   11
#define LONGITUDE_INDEX  15
#define CRC_INDEX        19

#define DEVICE_ID_SIZE   9
#define LATT_LONG_SIZE   4

static uint16_t  NODE_MODE     =  0;
static uint8_t   DEVICE_ID[10] = {0};
static uint16_t  NODE_ID       =  0;
static uint16_t  BUTTON_STATE  =  0;
static uint16_t  BATTERY_VOLT  =  0;
static uint32_t  LATITUDE      =  0;
static uint32_t  LONGITUDE     =  0;
static uint16_t  FINAL_BATTERY_VOLT  =  0;
bool Modbus_flag = 0;

uint16_t Convert_Volt_to_Percentage(void)
{
  uint8_t MIN_VOLT = 24;
  //uint8_t MAX_VOLT = 48;


if( BATTERY_VOLT != 0)
{
  float Temp_Bat_volt = BATTERY_VOLT;

  Temp_Bat_volt -= MIN_VOLT;

  Temp_Bat_volt /= MIN_VOLT;

  Temp_Bat_volt *= 100;

  FINAL_BATTERY_VOLT = Temp_Bat_volt;

  LOG(LL_INFO,("BATTERY_VOLT IN PERCENTAGE: %.2f ",Temp_Bat_volt));
}
return FINAL_BATTERY_VOLT;
}

/************** PARAMETER SET WRAPPERS  ***************/
void SET_NODE_MODE (uint8_t MD)
{
  NODE_MODE = 0;
  NODE_MODE = (MD & 0x80) >> 7;
  return;
}

void SET_DEVICE_ID (uint8_t *DI,uint8_t DI_Size)
{
  uint8_t SDI_i = 0;
  memset(DEVICE_ID,0,sizeof(DEVICE_ID));

  DEVICE_ID[SDI_i] = (DI[SDI_i] & 0x7F);
  SDI_i++;

  for(;SDI_i < DI_Size;SDI_i++)
  {
    DEVICE_ID[SDI_i] = DI[SDI_i];
  }
  
  return;
}

void SET_NODE_ID (uint8_t ID)
{
  NODE_ID = 0;
  NODE_ID = ID;
  return;
}

void SET_BATTERY_VOLT (uint8_t BV)
{
  BATTERY_VOLT = 0;
  BATTERY_VOLT = (BV & 0x3F);
  LOG(LL_INFO,("SET BATTERY_VOLT : %d ",BATTERY_VOLT));
  return;
}

void SET_BUTTON_STATE (uint8_t BS)
{
  BUTTON_STATE = 0;
  BUTTON_STATE = (BS & 0xC0) >> 6;
  return;
}

void SET_LATITUDE (uint8_t *LT,uint8_t LT_Size)
{
  LATITUDE = 0;
  for(uint8_t i = 0,j = 24;i < LT_Size;i++)
  {
    LATITUDE |= LT[i] << j;
    j -= 8;
  }
  return;
}

void SET_LONGITUDE (uint8_t *LN,uint8_t LN_Size)
{
  LONGITUDE = 0;
  for(uint8_t i = 0,j = 24;i < LN_Size;i++)
  {
    LONGITUDE |= LN[i] << j;
    j -= 8;
  }
  return;
}

/************** PARAMETER GET WRAPPERS  ***************/
uint8_t GET_NODE_MODE (void)
{
  return NODE_MODE;
}

uint8_t* GET_DEVICE_ID (void)
{
  return DEVICE_ID;
}

uint16_t GET_NODE_ID (void)
{
  return NODE_ID;
//  return 1050;
}

uint16_t GET_BATTERY_VOLT ( void)
{
  //return Convert_Volt_to_Percentage();
 return BATTERY_VOLT;
}

uint16_t GET_BUTTON_STATE (void)
{
  return BUTTON_STATE;
}

uint32_t GET_LATITUDE (void)
{
  return LATITUDE;
//  return 17123456;
}

uint32_t GET_LONGITUDE (void)
{
  return LONGITUDE;
//  return 78123456
}

void PRINT_LORA_PARAMETERS(void)
{
  LOG(LL_INFO,("LORA:NODE_MODE: %d -> %x",GET_NODE_MODE(),GET_NODE_MODE()));
  LOG(LL_INFO,("LORA:DEVICE_ID: %s",GET_DEVICE_ID()));
  LOG(LL_INFO,("LORA:NODE_ID: %d -> %x",GET_NODE_ID(),GET_NODE_ID()));
  LOG(LL_INFO,("LORA:BATTERY_VOLT: %d",GET_BATTERY_VOLT()));
  LOG(LL_INFO,("LORA:BUTTON: %d -> %x",GET_BUTTON_STATE(),GET_BUTTON_STATE()));
  LOG(LL_INFO,("LORA:LATITUDE: %d-> %x",GET_LATITUDE(),GET_LATITUDE()));
  LOG(LL_INFO,("LORA:LONGITUDE: %d-> %x",GET_LONGITUDE(),GET_LONGITUDE()));
  return;
}

void Reset_Lora_Parameters(void)
{
NODE_MODE = 0;
memset(DEVICE_ID,'\0',sizeof(DEVICE_ID));
NODE_ID = 0;
BATTERY_VOLT = 0;
BUTTON_STATE = 0;
LATITUDE  = 0;
LONGITUDE = 0;
FINAL_BATTERY_VOLT = 0;
}
void EXTRACT_LORA_PARAMETERS (uint8_t *Lora_pkt)
{
  SET_NODE_MODE(Lora_pkt[NODE_MODE_INDEX]);
  SET_DEVICE_ID(&Lora_pkt[NODE_MODE_INDEX],DEVICE_ID_SIZE);
  SET_NODE_ID(Lora_pkt[NODE_ID_INDEX]);
  SET_BUTTON_STATE(Lora_pkt[BUTTON_ST_INDEX]);
  SET_BATTERY_VOLT(Lora_pkt[BATTERY_VL_INDEX]);
  SET_LATITUDE(&Lora_pkt[LATITUDE_INDEX],LATT_LONG_SIZE);
  SET_LONGITUDE(&Lora_pkt[LONGITUDE_INDEX],LATT_LONG_SIZE);

  /********** Print Rxd lora parameters **************/
  PRINT_LORA_PARAMETERS();

}  

/**********************************************************************!
 * @fn          ModRTU_CRC
 * @brief       To caluclate crc
 * @param[in]   buf,len
 * @return      caluclated crc
 **********************************************************************/

uint16_t ModRTU_CRC(uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  
  for (uint8_t pos = 0; pos < len; pos++) 
	{
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
  
    for (uint8_t i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc; 

}

/**********************************************************************!
 * @fn          MODBUS_DATA_SEND
 * @brief       transmit data through modbus
 * @param[in]   void
 * @return      bool
 **********************************************************************/
uint32_t u32overTime = 0;
void MODBUS_DATA_SEND(uint8_t *modbus_txcmd_str, uint8_t modbus_txcmd_len)
{
    mgos_gpio_write(mgos_sys_config_get_modbus_de_re(),1);

if(mgos_uart_write_avail(mgos_sys_config_get_serial_num()))
{   
  //  LOG(LL_INFO, ("UART BYTES: %d",modbus_txcmd_len));
   /* LOG(LL_INFO, ("MODBUS DATA:"));
   for(int i = 0; i<modbus_txcmd_len;i++){

       LOG(LL_INFO, ("%x",modbus_txcmd_str[i]));
   }*/

  if(mgos_uart_write(mgos_sys_config_get_serial_num(),(uint8_t *)&modbus_txcmd_str[0], modbus_txcmd_len) == 0)
  {
   LOG(LL_INFO, ("UART DATA IS Not SENT"));
  }
  mgos_uart_flush(mgos_sys_config_get_serial_num());
  volatile uint32_t u32overTimeCountDown = u32overTime;
  while ( u32overTimeCountDown-- > 0);

  mgos_gpio_write( mgos_sys_config_get_modbus_de_re(),0);
  LOG(LL_INFO, ("UART DATA IS SENT"));
  
  memset(modbus_txcmd_str,'\0',modbus_txcmd_len);

}
}


//{0x01,0x03,0x0E,0x00,0x65,0x00,0x01,0x00,0x2D,0x06,0xB0,0x0D,0x80,0x1E,0x83,0x04,0x57,0x22,0xC1}

void TX_MODBUS_PACKET (void)
{
  static uint8_t Modbus_pkt_cntr = 0;
  static uint8_t Modbus_temp_pkt[20] = {0};

  memset(Modbus_Tx_Pkt,0,sizeof(Modbus_Tx_Pkt));
  memset(Modbus_temp_pkt,0,sizeof(Modbus_temp_pkt));
  Modbus_pkt_cntr = 0;

  /********************  Slave ID ***********************/
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = MODBUS_SLAVE_ID;                         // 0

  /********************  Function Code ***********************/
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = MODBUS_FUNC_COD;                         // 1

  /********************  Data Size ***********************/
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = 0;                                       // 2
  
  /********************  Node ID ***********************/
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_NODE_ID() & 0xFF00) >> 8;           // 3
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_NODE_ID() & 0x00FF);                // 4
  
  /********************  Button State ***********************/
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_BUTTON_STATE() & 0xFF00) >> 8;      // 5
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_BUTTON_STATE() & 0x00FF);           // 6
  
  /********************  Battery Voltage ***********************/
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_BATTERY_VOLT() & 0xFF00) >> 8;      // 7
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_BATTERY_VOLT() & 0x00FF);           // 8
  
  /********************  latitude ***********************/
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_LATITUDE() & 0xFF000000) >> 24;     // 9
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_LATITUDE() & 0x00FF0000) >> 16;     // 10
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_LATITUDE() & 0x0000FF00) >> 8;      // 11
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_LATITUDE() & 0x000000FF);           // 12
  
  /********************  longitude ***********************/
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_LONGITUDE() & 0xFF000000) >> 24;    // 13
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_LONGITUDE() & 0x00FF0000) >> 16;    // 14
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_LONGITUDE() & 0x0000FF00) >> 8;     // 15
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (GET_LONGITUDE() & 0x000000FF);          // 16

  /********************  Data Size ***********************/
  Modbus_Tx_Pkt[MODBUS_TXN_SIZE_INDEX] = Modbus_pkt_cntr - MODBUS_DATA_STT_INDEX;

  /********************  CRC  ****************************/
  uint16_t Modbus_CRC_Val =  ModRTU_CRC(&Modbus_Tx_Pkt[MODBUS_SLAVE_ID_INDEX],Modbus_pkt_cntr);
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (Modbus_CRC_Val & 0x00FF);          // 17
  Modbus_Tx_Pkt[Modbus_pkt_cntr++] = (Modbus_CRC_Val & 0xFF00) >> 8;               // 18

  /******************** Packet Transmiting ***************/
  MODBUS_DATA_SEND(Modbus_Tx_Pkt,Modbus_pkt_cntr);

  /******************** Test print ***********************/
  for(uint8_t i = 0; i < Modbus_pkt_cntr; i++)
  {
    LOG(LL_INFO, (" %x",Modbus_Tx_Pkt[i]));     
  }
  /********************************************************/
  return;  
}

/**********************************************************************!
 * @fn          Check_Device
 * @brief       To check the device STATIC or DYNAMIC
 * @param[in]   buf
 * @return      device status 1-> STATIC 0-> DYNAMIC
 **********************************************************************/

uint8_t Check_Device(uint8_t *buf)
{
  if(buf[0] & (1 << 7))
  {
    LOG(LL_INFO, ("DYNAMIC DEVICE"));

    return (0x01);
  }

  else
  {
    LOG(LL_INFO, ("STATIC DEVICE"));  
    return(0x00);
  }
  
}


/**********************************************************************!
 * @fn          Lora_Data_Chk_CRC
 * @brief       To check the CRC in lora recieved data
 * @param[in]   r_buf
 * @return      0 or 1
 **********************************************************************/

uint8_t Lora_Data_Chk_CRC(uint8_t *r_buf,uint8_t r_buf_len)
{
 	uint16_t rx_crc = (r_buf[r_buf_len - 2]) << 8;

	rx_crc |= r_buf[r_buf_len - 1];

  LOG(LL_INFO, ("Rxd CRC: %X",rx_crc));

	uint16_t cal_crc = ModRTU_CRC(&r_buf[0],(r_buf_len - 2));

  LOG(LL_INFO, ("Calc CRC: %X",cal_crc));
  if(cal_crc == rx_crc)
	{
		return 1;
	}
	return 0;
}

/**********************************************************************!
 * @fn          Send_Ack_To_Node
 * @brief       To send acknowledgement to the node
 * @param[in]   buffer
 * @return      void
 **********************************************************************/

void Send_Ack_To_Node(uint8_t* buffer)
{
  uint8_t Ack_txcmd_str[ACK_BUF_SIZE] ={'\0'};
  uint16_t MODBUS_CRC = 0;
  Ack_txcmd_str[0] = START_BYTE;                 //START BYTE
	Ack_txcmd_str[1] = GET_NODE_ID();         // Received NODE ID
	MODBUS_CRC = ModRTU_CRC(&Ack_txcmd_str[1],1);  // CRC FOR ACK
	Ack_txcmd_str[2] = (MODBUS_CRC & 0xFF00) >> 8 ;  // CRC MSB
	Ack_txcmd_str[3] = (MODBUS_CRC & 0x00FF);        // CRC LSB
  Ack_txcmd_str[4] = END_BYTE;                    // END BYTE

 LOG(LL_INFO, ("ACKNOWLEDGEMENT PACKET : %s",Ack_txcmd_str));

  mgos_send_lora(lora,(char *)Ack_txcmd_str);
}

 /**********************************************************************!
 * @fn          Validate_Rxed_Data
 * @brief       To validate lora received data
 * @param[in]   recvbuf
 * @return      void
 **********************************************************************/

 void Validate_Rxed_Data(uint8_t *rx_buf,uint8_t rx_buf_len) 
 {
   uint8_t cpy_recvbuf[LORA_BUF_SIZE]={'\0'}; 
   if((rx_buf[START_BYTE_INDEX] == START_BYTE) && (rx_buf[STOP_BYTE_INDEX] == END_BYTE)) // Checking start and end byte
   { 
    LOG(LL_INFO, ("START AND END BYTE MATCHED"));
    memset(cpy_recvbuf,0,sizeof(cpy_recvbuf));
    memcpy(cpy_recvbuf,&rx_buf[1],rx_buf_len-2);
    if(Lora_Data_Chk_CRC(cpy_recvbuf,rx_buf_len-2))
    {
      LOG(LL_INFO, ("CRC IS OK"));    
      Check_Device(cpy_recvbuf);
      EXTRACT_LORA_PARAMETERS(cpy_recvbuf);
      Send_Ack_To_Node(cpy_recvbuf);
      Modbus_flag = 1;
      mgos_lora_receive_mode(lora);
    }
   }
 }

/**********************************************************************!
 * @fn          lora_data_receive
 * @brief       Recieving Lora Packet 
 * @param[in]   void
 * @return      bool
 **********************************************************************/

void lora_data_receive(int packet_size)
{
      uint8_t recvbuf[LORA_BUF_SIZE] = {0};
      
        /*Receiving timer code */
   LOG(LL_INFO, ("\n[INFO]\t: LoRa Packet received"));
   LOG(LL_INFO, ("Packet size - %d",packet_size));
   
   if( packet_size == LORA_DATA_LENGTH)
   {
    for (int i = 0; i < packet_size; i++)
    {
      recvbuf[i] = (mgos_lora_read(lora));
    }

       LOG(LL_INFO, ("\n[INFO]\t: LoRa Packet : %s",recvbuf));

       Validate_Rxed_Data(recvbuf,packet_size);
   }

  /*  if(  mgos_mqtt_pubf(TELEMETRY_TOPIC, 1, 0,(const char*)recvbuf))
    {
          LOG(LL_INFO, ("MQTT published"));   
    }*/  
}


static void uart_dispatcher(int uart_nmbr, void *arg) 
{
  uint8_t rx_buf[15] = {0};
  size_t rx_av = mgos_uart_read_avail(uart_nmbr);
  //uint8_t uart_av = mgos_uart_read_avail(uart_nmbr);
  // LOG(LL_INFO, ("uart_av - %d",uart_av));  
  if (rx_av)
  {
     mgos_uart_read(uart_nmbr, rx_buf, rx_av);
     LOG(LL_INFO, ("Serial Data Received: %x %x %x %x %x %x %x %x", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]));
     TX_MODBUS_PACKET(); 

     Reset_Lora_Parameters();
//     MODBUS_DATA_SEND(Modbus_Tx_Pkt,19);
  }
 (void)arg;
}

static void timer_cb(void *arg)
{
  if(Modbus_flag == 1)
  {
    TX_MODBUS_PACKET(); 
    Modbus_flag = 0;
  }
  (void) arg;
}

/**********************************************************************!
 * @fn          mgos_app_init_result
 * @brief       Initialization function 
 * @param[in]   void
 * @return      bool
 **********************************************************************/

enum mgos_app_init_result mgos_app_init(void) {
    
     /********************LORA CONFIGURATION*******************************/  

  lora = mgos_lora_create();

mgos_lora_setpins(lora,mgos_sys_config_get_spi_cs0_gpio(), mgos_sys_config_get_lora_reset(),mgos_sys_config_get_lora_dio0());

long int frequency = mgos_sys_config_get_lora_frequency();

 if(!mgos_lora_begin(lora,frequency))
  {
    LOG(LL_INFO,("[INFO]:LORA NOT INTIALIZED"));
  }
  else
  {
     LOG(LL_INFO,("[INFO]:LORA INTIALIZED"));
  }

  mgos_lora_receive_callback(lora,lora_data_receive);
  mgos_lora_receive_mode(lora);

  /********************UART CONFIGURATION*******************************/  

  struct mgos_uart_config ucfg;                                          
  mgos_uart_config_set_defaults(mgos_sys_config_get_serial_num(), &ucfg);

  ucfg.baud_rate = mgos_sys_config_get_serial_baud_rate();
  ucfg.rx_buf_size = mgos_sys_config_get_serial_rx_buf_size();
  ucfg.tx_buf_size =  mgos_sys_config_get_serial_tx_buf_size();

   if (!mgos_uart_configure(mgos_sys_config_get_serial_num(), &ucfg)) {
  LOG(LL_ERROR, ("Failed to configure UART%d", mgos_sys_config_get_serial_num()));
  }

   mgos_gpio_set_mode(mgos_sys_config_get_modbus_de_re(),MGOS_GPIO_MODE_OUTPUT);
   mgos_gpio_write( mgos_sys_config_get_modbus_de_re(),0);

   mgos_uart_set_dispatcher(mgos_sys_config_get_serial_num(), uart_dispatcher, NULL /* arg */);

  /* Controls whether UART receiver is enabled. */
  mgos_uart_set_rx_enabled(mgos_sys_config_get_serial_num(), true);

  if (!(mgos_uart_is_rx_enabled(mgos_sys_config_get_serial_num())))
  {
    LOG(LL_ERROR, ("UART%d Receiver not Enabled", mgos_sys_config_get_serial_num()));
  //  return false;
  }
if(mgos_sys_config_get_modbus_annunciator_enable())
{
 mgos_set_timer(100 /*ms*/, MGOS_TIMER_REPEAT, timer_cb, NULL);  
}
  return MGOS_APP_INIT_SUCCESS;
}
