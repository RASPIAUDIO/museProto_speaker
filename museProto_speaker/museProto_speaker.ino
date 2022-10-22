/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
//////////////////////////////////////////////////////////////////////////
//
// MuseProto board speaker
//
//////////////////////////////////////////////////////////////////////////
#include "Audio.h"
#include "SD.h"
#include "FS.h"


extern "C"
{
#include "hal_i2c.h"
}

#include "Arduino.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <sys/socket.h>
#include <dirent.h>
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "SPIFFS.h"



#define I2S_DOUT      26
#define I2S_BCLK      5
#define I2S_LRC       25
#define I2S_DIN       35
#define I2SN (i2s_port_t)0
#define I2CN (i2c_port_t)0
#define SDA 18
#define SCL 23

//Buttons
#define MU GPIO_NUM_12      // short => mute/unmute  long => stop (deep sleep)
#define VM GPIO_NUM_32      // short => volume down  long => previous station
#define VP GPIO_NUM_19      // short => volume up   long => next station
#define STOP GPIO_NUM_12    // for wake up
#define GAIN GPIO_NUM_23

//Amp power enable
#define PW GPIO_NUM_21    

// Sd detect 
#define SDD GPIO_NUM_34     

#define SD_CS         13
#define SPI_MOSI      15
#define SPI_MISO      2
#define SPI_SCK       14



#define maxMode 2
#define btM 0
#define sdM 1

#define maxVol 40

uint8_t vplus = 0, vmoins = 0, vmode = 0;
uint8_t vmute = 0;
bool mute = false;
uint8_t vfwd = 0;
uint8_t vbck = 0;
bool dofwd = false;
bool dobck = false;
uint8_t  vauxd, vsdd;

int vol,oldVol,oldVolM;
int mode = btM;
bool sdON = false;
bool beepON = false;
uint32_t sampleRate;
bool muteON = false;

static uint32_t m_pkt_cnt = 0;
static esp_a2d_audio_state_t m_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;

#define TAG "bt_sp"
// typedef int (*http_data_cb) (http_parser*, const char *at, size_t length);
// typedef int (*http_cb) (http_parser*);

Audio audio;

static File root;
static File file;
static bool mp3ON;

#define BLOCK_SIZE 128
#define I2SR (i2s_port_t)0

 const i2s_config_t i2s_configR = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX ), //  transfer
      .sample_rate = 44100,                     
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, //
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      .dma_buf_count = 4,                           // number of buffers
      .dma_buf_len = BLOCK_SIZE                     // samples per buffer
  };
  
      i2s_pin_config_t pin_configR=
      {
      .bck_io_num = I2S_BCLK,    // BCKL
      .ws_io_num = I2S_LRC ,    // LRCL
      .data_out_num = I2S_DOUT,  // DOUT
      .data_in_num = I2S_DIN    // DIN
      };

   const int16_t volumetable[22]={0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,34,38,44,50,55,60,63}; //22 elements
//////////////////////////////////////////////////////////////////////////
//
// plays .wav records (in SPIFFS)
//////////////////////////////////////////////////////////////////////////
void playWav(char* n)
{
  struct header
  {
    uint8_t a[16];
    uint8_t cksize[4];
    uint8_t wFormatTag[2];
    uint8_t nChannels[2];
    uint8_t nSamplesPerSec[4];
    uint8_t c[16];  
  };
   uint32_t rate;
   uint8_t b[44];
   int l;
   bool mono;
   size_t t;
// read file header   
   File f = SPIFFS.open(n, FILE_READ);
   l = (int) f.read(b, sizeof(b));
//inits   
   if (b[22] == 1) mono = true; else mono = false;  
   rate =  (b[25] << 8) + b[24];
   printf(" rate = %d\n",rate);
   i2s_set_clk(I2SN, rate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
//
   i2s_zero_dma_buffer((i2s_port_t)0); 
//plays the record   
   do
   {
    if(mono == true)
    {
       l = (int)f.read((uint8_t*)b, 2);
       b[2] = b[0]; b[3] = b[1];
       
    }
    else      
       l = (int)f.read((uint8_t*)b, 4);   
             
   i2s_write(I2SN, b, 4, &t,1000);
   }
   while(l != 0);
   i2s_zero_dma_buffer((i2s_port_t)0);
   f.close();
}


/////////////////////////////////////////////////////////////////////
// beep....
/////////////////////////////////////////////////////////////////////
void beep(void)
{
  return; ////////////////////////////////////////////////////////////////////////
#define volBeep 15 
  audio.setVolume(volBeep);
  beepON = true;
  playWav("/Beep.wav");
  beepON = false;
  i2s_set_clk(I2SN, sampleRate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
}

//////////////////////////////////////////////////////////////////////
// annonce tiny messages (2-3 sec)
//////////////////////////////////////////////////////////////////////
void modeCall(void)
{
  char*n[] = {"/bluetooth.wav", "/player.wav"};
  printf("%d  ====> %s\n", mode, n[mode]);
  beepON = true;
  playWav(n[mode]);
  beepON = false;
  i2s_set_clk(I2SN, sampleRate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
}
////////////////////////////////////////////////////////////////////
//
// task managing playlist on SD (SPI)
//
////////////////////////////////////////////////////////////////////
static void sd(void* pdata)
{ 
    static int N;
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    delay(500);
    if(!SD.begin(SD_CS))printf("init. SD failed !\n");
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    
    root = SD.open("/");
    file = root.openNextFile();

    Serial.println("nom du fichier");
    Serial.println(file.name());
    
    audio.connecttoSD(file.name());
    audio.setVolume(vol);               
    mp3ON = true;
    N = 0;
    while(1)
    {
//////////////////////////////////////////      
// next file  
/////////////////////////////    
       if(mp3ON == false)
       { 
           mp3ON = true;
           file = root.openNextFile(); 
           if(file)
           {
              N++;
              Serial.println(file.name());
              audio.connecttoSD(file.name());
           }
           else
           {
// playing list end 
// restart at the first song
              root.rewindDirectory();
              file = root.openNextFile();
              if(file)
              {
              N = 0;
              Serial.println(file.name());            
              audio.connecttoSD(file.name());   
              }  
              else
              {
// error SD mode stopped                
              sdON = 0;
              audio.stopSong();
              SPI.end();
              SD.end();
              vTaskDelete(NULL);        
              }
           }
        } 
/////////////////////////////////////
//Forward  
//////////////////////////////////      
        if(dofwd == true)
        {
          audio.stopSong();
          file = root.openNextFile();
          if(file)
           {
              N++;
              printf("%d\n",N);
              Serial.println(file.name());
              audio.connecttoSD(file.name());
           }
           else
           {
// playing list end 
// restart at the first song             
              root.rewindDirectory();
              file = root.openNextFile();
              if(file)
              {
              N = 0;
              Serial.println(file.name());            
              audio.connecttoSD(file.name());   
              }  
              else
              {
// error SD mode stopped                   
              sdON = 0;
              audio.stopSong();
              SPI.end();
              SD.end();
              vTaskDelete(NULL);        
              }       
           }
           dofwd = false;
        }
///////////////////////////////////////////////////
//Backward
/////////////////////////////////////
        if(dobck == true)
        {
          audio.stopSong();
          N--;
          if(N < 0) N = 0;
//rewind 
//going to song #N
          root.rewindDirectory();     
          for(int i=0;i<=N;i++)
          {
            file = root.openNextFile();   
          }
          Serial.println(file.name());            
          audio.connecttoSD(file.name());
          dobck = false   ;
        }


///////////////////////////////////////////        
//  file playing  
//////////////////////////////  
       if((mute == false) && (beepON == false))audio.loop(); 
////////////////////////////////////       
// mode change  
/////////////////////////////     
       if(mode != sdM)
       {
              sdON = 0;
              audio.stopSong();
              SPI.end();
              SD.end();
              vTaskDelete(NULL);
       }      
    }
}
/////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////
//
// bluetooth callbacks
///////////////////////////////////////////////////////////////////////////
void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);
    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;
    printf("================> %s\n",(char*) attr_text);

     rc->meta_rsp.attr_text = attr_text;
}

void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    printf("top\n");
  
    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        bt_app_alloc_meta_buffer(param);
        break;
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        esp_avrc_ct_send_metadata_cmd(0,ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
        break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
//        bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

void bt_app_a2d_data_cb(const uint8_t *d, uint32_t len)
{
    int L;
    int16_t s[2];
    if(mode != btM) return;
    if(beepON == true) return;  
    
// Volume 
// 1- each 16 bits sample (right &  left) is  extracted from the 8 bits data stream (little endian !)
// 2- each sample is modified according to the volume ==> s * volume / maxvolume   
//   printf("%d \n",len);
    L = (len/4)*4;
    for(int i=0;i<L;i=i+4)
    {
      s[0] = (((uint16_t)d[i+1]) << 8) + (uint16_t)d[i];
      s[1] = (((uint16_t)d[i+3]) << 8) + (uint16_t)d[i+2]; 
      
      s[0] = (s[0]*volumetable[vol]) >> 6;
      s[1] = (s[1]*volumetable[vol]) >> 6;     
      
      i2s_write_bytes(I2SN, (const char *)s, 4, portMAX_DELAY);   
    }    
      if (++m_pkt_cnt % 100 == 0) {
        ESP_LOGE(BT_AV_TAG, "audio data pkt cnt %u", m_pkt_cnt);
    }
}

// callback for A2DP sink 
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *p_param)
{
   if(mode != btM) return;
   ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp conn_state_cb, state %d", a2d->conn_stat.state);
        break;
    }
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp audio_state_cb state %d", a2d->audio_stat.state);
        m_audio_state = a2d->audio_stat.state;
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            m_pkt_cnt = 0;
    case ESP_A2D_AUDIO_STATE_EVT: {
        }
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp audio_cfg_cb , codec type %d", a2d->audio_cfg.mcc.type);
        // for now only SBC stream is supported
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            int sample_rate = 16000;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6)) {
                sample_rate = 32000;
            } else if (oct0 & (0x01 << 5)) {
                sample_rate = 44100;
            } else if (oct0 & (0x01 << 4)) {
                sample_rate = 48000;
            }
            sampleRate = sample_rate;
            i2s_set_clk(I2SN, sample_rate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
        
            ESP_LOGI(BT_AV_TAG, "configure audio player %x-%x-%x-%x\n",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            ESP_LOGI(BT_AV_TAG, "audio player configured, samplerate=%d", sample_rate);
        }
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }  
}
/////////////////////////////////////////////////////////////////////////////////////////
//
// APP init
/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(115200);
    esp_err_t err;
   // err = nvs_flash_init();
   // if(err != ESP_OK) printf("nvs flash error...\n");
 
    if(!SPIFFS.begin(true)){Serial.println("Erreur SPIFFS");   
    printf("====> %d\n",(int)SPIFFS.totalBytes());
    printf("====> %d\n",(int)SPIFFS.usedBytes());   
    SPIFFS.format();  
    }
    printf(" SPIFFS used bytes  ====> %d of %d\n",(int)SPIFFS.usedBytes(), (int)SPIFFS.totalBytes());      

    vol = oldVol = maxVol/2;

///////////////////////////////////////////////////////   
// initi gpios
////////////////////////////////////////////////////////////
//gpio_reset_pin
        gpio_reset_pin(MU);
        gpio_reset_pin(VP);
        gpio_reset_pin(VM);
        gpio_reset_pin(SDD);
//       gpio_reset_pin(STOP);      
         
//gpio_set_direction
        gpio_set_direction(MU, GPIO_MODE_INPUT);  
        gpio_set_direction(VP, GPIO_MODE_INPUT);  
        gpio_set_direction(VM, GPIO_MODE_INPUT);          
        gpio_set_direction(SDD, GPIO_MODE_INPUT);  

//gpio_set_pull_mode
        gpio_set_pull_mode(MU, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(VP, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(VM, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(SDD, GPIO_PULLUP_ONLY);


// power enable
        gpio_reset_pin(PW);
        gpio_set_direction(PW, GPIO_MODE_OUTPUT);        
        gpio_set_level(PW, 1); 
   
// set amp gain
   gpio_set_pull_mode(GAIN, GPIO_PULLDOWN_ONLY);   // 15dB  
                       
// init i2s default rates
 
   i2s_driver_install(I2SR, &i2s_configR,0,NULL);
   i2s_set_pin(I2SR, &pin_configR);
   i2s_set_clk(I2SN, 44100, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
   sampleRate = 44100;

/////////////////////////////////////////////////////////////////////////
// init bluetooth
//    
/////////////////////////////////////////////////////////////////////////
   
    btStart();
    
    if (esp_bluedroid_init() != ESP_OK) {
        Serial.println("initialize bluedroid failed!");
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        Serial.println("enable bluedroid failed!");
        return;
    }  
    uint8_t mac[6];
    char dev_name[30 ];
    esp_read_mac((uint8_t *)&mac, ESP_MAC_WIFI_STA);
    sprintf(dev_name, "MUSE_SPEAKER-%x%x%x",mac[3],mac[4],mac[5]);
    printf("%s\n",dev_name);
    esp_bt_dev_set_device_name(dev_name);

//initialize A2DP sink 
    esp_a2d_register_callback(&bt_app_a2d_cb);
    esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
    esp_a2d_sink_init();
//initialize AVRC controller
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
//set discoverable and connectable mode, wait to be connected 
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
    esp_avrc_ct_send_metadata_cmd(0,ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
 
    printf("starting \n");
    mode = btM;
 //////////////////////////////////////////////////////////////////////   
   modeCall();  
}
   void loop(){
#define longPress  8 
#define veryLongPress 20
static int v0, v1, v2;
static int ec0=0, ec1=0, ec2=0;
static int b0 = -1, b1 = -1, b2 = -1;

     delay(100);



/////////////////////////////////////////////////////////////////  
// buttons handling  => bn
//         bn = time the button n remains pressed (x 100 ms)
/////////////////////////////////////////////////////////////////
    if((gpio_get_level(VP) == 1) && (ec0 == 1)){b0 = v0; ec0 = 0;}
    if((gpio_get_level(VP) == 1) && (b0 == -1)) {v0 = 0; ec0 = 0;}
    if(gpio_get_level(VP) == 0) {v0++; ec0 = 1;}
   
    if((gpio_get_level(VM) == 1) && (ec1 == 1)){b1 = v1; ec1 = 0;}
    if((gpio_get_level(VM) == 1) && (b1 == -1)) {v1 = 0; ec1 = 0;}
    if(gpio_get_level(VM) == 0) {v1++; ec1 = 1;}
   
    if((gpio_get_level(MU) == 1) && (ec2 == 1)){b2 = v2; ec2 = 0;}
    if((gpio_get_level(MU) == 1) && (b2 == -1)) {v2 = 0; ec2 = 0;}
    if(gpio_get_level(MU) == 0) {v2++; ec2 = 1;}

// Volume + (VP short) Volume - (VM short)
    if(muteON == false)
    {
    oldVol = vol;
    if((b0 > 0) && (b0 < longPress)) {vol++ ;b0 = -1;printf("P\n");}
    if((b1 > 0) && (b1 < longPress)) {vol-- ;b1 = -1;printf("M\n");}

    if (vol > maxVol) vol = maxVol;
    if (vol < 0) vol = 0;


//volume change
   if((vol != oldVol))
  {
      beep();
      oldVol = vol;  
      printf("vol = %d\n",vol);
      audio.setVolume(vol);
      char b[4];
      sprintf(b,"%02d",vol);     
      File ln = SPIFFS.open("/volume", "w");
      ln.write((uint8_t*)b, 2);
      ln.close();     
   }
    }

// mute / unmute   (MU short)
   if((b2 > 0) && (b2 < longPress))
   {
    if(muteON == false)
    {
       oldVolM = vol;
       vol = 0;
       audio.setVolume(vol);
       printf("mute on\n");
       muteON = true;           
    }
    else
    {
       vol = oldVolM;
       audio.setVolume(vol);
       printf("mute off\n");                                     
       muteON = false;
    }
    b2 = -1; 
   }

// stop (MU very long)   
  if((b2 > 0) && (b2 > veryLongPress))
    {
      b2 = -1;
      esp_sleep_enable_ext0_wakeup(STOP,LOW);    
      esp_deep_sleep_start();
    }     
      
//      
     
// forward / backward    (for SD)
      if((b0 > 0) && (b0 > longPress) && (mode == sdM)) {dofwd = true; b0 = -1;}
      if((b1 > 0) && (b1 > longPress) && (mode == sdM)) {dobck = true; b1 = -1;}



// SD detect
      if(gpio_get_level(SDD) == 1) 
      {
      mode = sdM;
      if(sdON == false){modeCall(); xTaskCreate(sd, "playlist", 5000, NULL, 1, NULL); sdON = true;}
      }
      else    
      if(sdON == true) {audio.stopSong(); mode = btM; modeCall();sdON = false;}
}


    
// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
    if(strstr(info, "SampleRate=") > 0) 
    {
    sscanf(info,"SampleRate=%d",&sampleRate);
    printf("==================>>>>>>>>>>%d\n", sampleRate);
    }
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print("eof_mp3     ");Serial.println(info);mp3ON = false;
}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreaminfo(const char *info){
    Serial.print("streaminfo  ");Serial.println(info);
    Serial.println("top");
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
    
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print("eof_speech  ");Serial.println(info);
}
