#include <Arduino.h>

#include <M5StickCPlus.h>
#include <driver/i2s.h>

#include "cindex.h"
#include "findfun.h"

#include <Preferences.h>

#define DEBUG true

#define PIN_CLK     0
#define PIN_DATA    34

#define DISPLAY_GAIN_FACTOR 100
#define DISPLAY_HEIGHT 135
#define DISPLAY_WIDTH 240

#define AUDIO_SAMPLE_HZ 50000u
#define AUDIO_SAMPLE_MILLIS 100u
#define AUDIO_SAMPLE_QTY 5000u    // 16 bit (AUDIO_SAMPLE_HZ / AUDIO_SAMPLE_MILLIS)
#define AUDIO_DEAD_HBAND 20       // One side amplitude dead band for zero crossing Schmitt trigger (test for it)
#define AUDIO_SAMPLE_SCALE 100

#define I2S_READ_LEN    (AUDIO_SAMPLE_QTY*2)
#define I2S_BUF_QTY 20
#define I2S_BUF_LEN 1000           // 8 bit (I2S_BUF_QTY * I2S_BUF_LEN / 2 == I2S_READ_LEN)


#define NUM_POLES_FILTER 1


#if DEBUG
# define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
# define DEBUG_PRINT(...)
#endif

TFT_eSprite waveDispBuff = TFT_eSprite(&M5.Lcd);
TFT_eSprite msgBuff = TFT_eSprite(&M5.Lcd);

TaskHandle_t Task_handle_record;
TaskHandle_t Task_handle_findfun;

TickType_t xLastWakeTime;

uint8_t i2s_buffer[I2S_READ_LEN] = {0};
int16_t filtered_wave[AUDIO_SAMPLE_QTY] = {0};

uint8_t oldy[DISPLAY_WIDTH];
int16_t *adcBuffer = NULL;

#define NUM_FILTER_SAMPLES 10
uint16_t filter_history[NUM_FILTER_SAMPLES] = {0};
cindex fh_idx(NUM_FILTER_SAMPLES);


float findfun_concert_pitch = 440.0; // concert pitch (piano middle A : midi 69 : A4)
// sampling frequency (hz)  for generating integer period lengths 
float findfun_sample_hz = AUDIO_SAMPLE_HZ * AUDIO_SAMPLE_SCALE; // same as the sample frequency of the edges sent    
int findfun_lmi = 21 ; // guitar 38 # piano 21
int findfun_hmi = 96 + 1; // guitar 66 # piano 96
int findfun_ncs = 20 ; // number of correlation shifts : increase for more precision but also workload
int infered_midi_cent_idx = 0;

vector<int> wave_edge_ticks;

bool wave_ready = false;
bool inference_ready = false;


int x_zero_cross(int x0,int y0,int x1,int y1){

    float m = (y1-y0)/(x1-x0);
    int x = m*(0 - y0) + x0;
    return x;
}

void filterWave(){

#if (NUM_POLES_FILTER == 0)

  for (int n = NUM_POLES_FILTER; n < AUDIO_SAMPLE_QTY; n++) {
      filtered_wave[n] =  adcBuffer[n];
  }

#elif (NUM_POLES_FILTER == 1)

 
    // first order recursive high pass coefficients
    float x = 0.99;
    float a0 = (1 + x) / 2;
    float a1 = -(1 + x) / 2;
    float b1 = x;
  

    for (int n = NUM_POLES_FILTER; n < AUDIO_SAMPLE_QTY; n++) {
        // remove dc using first order recursive high pass filter
        filtered_wave[n] =  a0*adcBuffer[n] + 
                            a1*adcBuffer[n-1] + 
                            b1*filtered_wave[n-1];       

  }

    // first order recursive low pass coefiicients
   
   x = 0.0035;
   a0 = 1 - x;
   b1 = x;

    for (int n = NUM_POLES_FILTER; n < AUDIO_SAMPLE_QTY; n++) {
        // remove dc using first order recursive high pass filter
        filtered_wave[n] =  a0*filtered_wave[n] + 
                            b1*filtered_wave[n-1];

  }


#endif

}

void showWave() {
    waveDispBuff.fillRect(0, 0, 240, 135, WHITE);
    int y;
    for (int n = 0; n < DISPLAY_WIDTH; n++) {
        y = filtered_wave[n + NUM_POLES_FILTER] * DISPLAY_GAIN_FACTOR;
        y = map(y, INT16_MIN, INT16_MAX, 10, DISPLAY_HEIGHT-10);
        waveDispBuff.drawPixel(n, y, BLACK);
        oldy[n] = y;
    }

    for (auto wet: wave_edge_ticks){
        auto swet = wet / AUDIO_SAMPLE_SCALE;
        if(swet < DISPLAY_WIDTH){
            waveDispBuff.drawLine(swet , 0, swet , DISPLAY_HEIGHT, BLUE);
        }
    }

    if(inference_ready){
        inference_ready = false;

    }
    waveDispBuff.drawLine(0, DISPLAY_HEIGHT / 2, DISPLAY_WIDTH, DISPLAY_HEIGHT / 2, GREEN);
    waveDispBuff.setCursor(10,10,1);
    waveDispBuff.print(infered_midi_cent_idx);
    waveDispBuff.pushSprite(0, 0);
}

void mic_record_task(void *arg) {
    DEBUG_PRINT("mic_record_task() running on core %d\n", xPortGetCoreID());
    size_t bytesread;
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil( &xLastWakeTime, AUDIO_SAMPLE_MILLIS / portTICK_RATE_MS );
        i2s_read(I2S_NUM_0, (char *)i2s_buffer, I2S_READ_LEN, &bytesread,
                 (AUDIO_SAMPLE_MILLIS / portTICK_RATE_MS));
        adcBuffer = (int16_t *)i2s_buffer;
        filterWave();
        showWave();
        wave_ready = true;
    }
}

void findfun_task(void *arg){
    DEBUG_PRINT("findfun_task() running on core %d\n", xPortGetCoreID());

    findfun ff = findfun();
    ff.begin(   findfun_concert_pitch,
                findfun_sample_hz,
                findfun_lmi,
                findfun_hmi,
                findfun_ncs);

    while(1) {
        
        if(wave_ready){
            wave_ready =  false;

            int comparator_state = 0;
            DEBUG_PRINT("wave_edge_ticks size before: %d\n", wave_edge_ticks.size());
            wave_edge_ticks.clear();
            DEBUG_PRINT("wave_edge_ticks size after: %d\n", wave_edge_ticks.size());

            // todo - scale sample frequency by 1000 and linear interpolate zero crossing

            int x0= 0, x1 = 0, y0 = 0, y1 = 0, xzc = 0;
            int num_cross = 0;
            int num_cross_pass = 2;
            bool crossed = false;

            for (int n = NUM_POLES_FILTER; n < AUDIO_SAMPLE_QTY; n++) {

                // detect edges with schmitt trigger and push to vector
                if(filtered_wave[n] > AUDIO_DEAD_HBAND){
                    if(filtered_wave[n-1] <= AUDIO_DEAD_HBAND){
                        if(comparator_state != 1){
                            comparator_state = 1;
                            crossed = true;
                        }
                    }
                } 
                else if(filtered_wave[n] < (0-AUDIO_DEAD_HBAND)){
                    if(filtered_wave[n-1] >= (0-AUDIO_DEAD_HBAND)){
                        if(comparator_state != (-1)){
                            comparator_state = (-1);
                            crossed = true;
                        }
                    }
                } 

                if (crossed){
                    num_cross++;
                    if( num_cross >= num_cross_pass){
                        crossed = false;
                        x1 = n * AUDIO_SAMPLE_SCALE;
                        x0 = (n - 1) * AUDIO_SAMPLE_SCALE;
                        y1 = filtered_wave[n];
                        y0 = filtered_wave[n-1];
                        xzc = x_zero_cross(x0, y0, x1, y1);
                        wave_edge_ticks.push_back(xzc);
                    }
                }


            }
            vector<int> test_vector = {22677, 	45404, 	68131, 	90858, 	113585, 	136312, 	159039 }; // 4500 midicent
            infered_midi_cent_idx = ff.find_midi_cent(wave_edge_ticks);
            inference_ready = true;
        }
        vTaskDelay( 10 / portTICK_RATE_MS );
    }
}

void i2sInit() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = AUDIO_SAMPLE_HZ,
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count    = I2S_BUF_QTY,
        .dma_buf_len      = I2S_BUF_LEN,
    };

    i2s_pin_config_t pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

    pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num    = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num  = PIN_DATA;

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, AUDIO_SAMPLE_HZ, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);
    waveDispBuff.fillScreen(WHITE);
    waveDispBuff.setTextColor(RED);
    waveDispBuff.setTextSize(4);
    waveDispBuff.createSprite(DISPLAY_WIDTH, DISPLAY_HEIGHT);

    delay(500);
    DEBUG_PRINT("setup() running on core %d\n", xPortGetCoreID());

    i2sInit();
    // xTaskCreate(mic_record_task, "mic_record_task", 2048, NULL, 1, NULL);
    xTaskCreatePinnedToCore(
      mic_record_task, /* Function to implement the task */
      "Task_record", /* Name of the task */
      (21)*1024,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task_handle_record,  /* Task handle. */
      1); /* Core where the task should run */

    xTaskCreatePinnedToCore(
      findfun_task, /* Function to implement the task */
      "Task_findfun", /* Name of the task */
      (4)*1024,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &Task_handle_findfun,  /* Task handle. */
      0); /* Core where the task should run */

}

void loop() {

    DEBUG_PRINT("loop() running on core %d\n", xPortGetCoreID());

    vTaskDelay(1000 / portTICK_RATE_MS);  // otherwise the main task wastes half
                                          // of the cpu cycles
}