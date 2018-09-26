#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "soc/syscon_reg.h"
#include "driver/adc.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "WiFi.h"
// #include "dac_cosine.c"

#define TAG "adc_i2s"

// Physically connect pins 25 and 34

uint32_t SAMPLE_RATE = 120E3; //44100;//120E3;
uint32_t NUM_SAMPLES = 64;
uint32_t SIGNAL_FREQ = 10E3;

#define WIFI_SSID ("BWQ")
#define WIFI_PASSWORD ("testpass")
WiFiServer server(80);

//declare socket related variables
#include <WebSocketServer.h>
WebSocketServer webSocketServer;

#define READ_PIN 32
#define LED 2

static QueueHandle_t i2s_event_queue;
static EventGroupHandle_t wifi_event_group;

bool streaming = false;

// init wifi with access point mode
void initWifi()
{

    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Broadcasting AP SSID: ");
    Serial.println(WIFI_SSID);

    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(ip);

    server.begin();
}

// init Serial comms
void initSerial()
{
    Serial.begin(115200);
    //921600
    //115200
}

// init gpio for led and other functions
void initGpio()
{
    pinMode(LED, OUTPUT);
}

void setup()
{
    initSerial();
    initGpio();
    initWifi();

    i2s_config_t i2s_config;
    i2s_config.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
    i2s_config.sample_rate = SAMPLE_RATE;                   // 120 KHz
    i2s_config.dma_buf_len = NUM_SAMPLES;                   // 64
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT; // Should be mono but doesn't seem to be
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT; // I2S_BITS_PER_SAMPLE_8BIT    I2S_BITS_PER_SAMPLE_16BIT    I2S_BITS_PER_SAMPLE_24BIT    I2S_BITS_PER_SAMPLE_32BIT
    i2s_config.use_apll = true,
    i2s_config.fixed_mclk = SAMPLE_RATE;
    i2s_config.communication_format = I2S_COMM_FORMAT_I2S;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = 8;
    //install and start i2s driver
    i2s_driver_install(I2S_NUM_0, &i2s_config, 1, &i2s_event_queue);
    // Connect ADC to I2S
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);

    // Generate test signal
    int step = nearbyint(0.008192 * SIGNAL_FREQ); // ~10kHz
    dac_cosine_enable(DAC_CHANNEL_1);             // enable the generator
    dac_frequency_set(0, step);                   // frequency setting is common to both channels
    dac_scale_set(DAC_CHANNEL_1, 0);              // No scaling (0~3.7v)
    dac_offset_set(DAC_CHANNEL_1, 0);             // No Offest
    dac_invert_set(DAC_CHANNEL_1, 2);             // Sinewave
    dac_output_enable(DAC_CHANNEL_1);             // Start DAC

    i2s_adc_enable(I2S_NUM_0); // Start ADC
}

void loop()
{

    WiFiClient client = server.available();
    if (client.connected() && webSocketServer.handshake(client))
    {

        IPAddress ip = client.remoteIP();
        Serial.print("Connecting to: ");
        Serial.println(ip);

        String data;
        while (client.connected())
        {

            if (streaming)
            {

                uint16_t i2s_read_buff[NUM_SAMPLES];
                system_event_t evt;
                if (xQueueReceive(i2s_event_queue, &evt, portMAX_DELAY) == pdPASS)
                {
                    if (evt.event_id == 2)
                    {
                        i2s_read_bytes(I2S_NUM_0, (char *)i2s_read_buff, NUM_SAMPLES * 2, portMAX_DELAY);
                        for (int i = 0; i < NUM_SAMPLES; i++)
                        {
                            Serial.printf("%X, ", i2s_read_buff[i] & 0xFFF);
                            webSocketServer.sendData(String(i2s_read_buff[i] & 0xFFF));
                        }
                        webSocketServer.sendData("\n");
                        Serial.println();
                    }
                }
            }

            data = webSocketServer.getData();
            if (data.length() > 0)
            {
                Serial.println(data);
                if (data == "start")
                {
                    digitalWrite(LED, HIGH);
                    streaming = true;
                }
                else if (data == "stop")
                {
                    digitalWrite(LED, LOW);
                    streaming = false;
                }
            }
        }

        Serial.print("Disconnecting from: ");
        Serial.println(ip);
        delay(100);
    }
}
