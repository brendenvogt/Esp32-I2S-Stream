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

#include <iostream>
#include <sstream>
#include <string>

#define TAG "adc_i2s"

//i2s debug
#define I2S_BUF_DEBUG (1)
//i2s number
#define I2S_NUM (i2s_port_t(0))
//i2s sample rate
#define I2S_SAMPLE_RATE (48000)
//i2s data bits
#define I2S_SAMPLE_BITS (32)
//I2S Buffer length
#define I2S_BUFFER_LEN (512)
//I2S Buffer length
#define I2S_BUFFER_COUNT (2)
//I2S read buffer length sample size times the number of samples
#define I2S_READ_LEN (I2S_SAMPLE_BITS * I2S_BUFFER_LEN)
//I2S data format
// #define I2S_FORMAT (I2S_CHANNEL_FMT_RIGHT_LEFT)
#define I2S_FORMAT (I2S_CHANNEL_FMT_ONLY_RIGHT)

//I2S channel number
#define I2S_CHANNEL_NUM ((I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
//I2S built-in ADC unit
#define I2S_ADC_UNIT ADC_UNIT_1
//I2S built-in ADC channel alt: ADC1_CHANNEL_6 or ADC1_CHANNEL_0
#define I2S_ADC_CHANNEL ADC1_CHANNEL_6

//Wifi SSID
#define WIFI_SSID ("Esp32")
//Wifi passwdord
#define WIFI_PASSWORD ("testpass")
WiFiServer server(80);

//declare socket related variables
#include <WebSocketServer.h>
WebSocketServer webSocketServer;

#define LED 2

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
    Serial.begin(115200); //921600//115200
}

// init gpio for led and other functions
void initGpio()
{
    pinMode(LED, OUTPUT);
}

void initI2S()
{
    i2s_config_t i2s_config;
    i2s_config.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
    i2s_config.sample_rate = I2S_SAMPLE_RATE;
    i2s_config.bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS);
    i2s_config.dma_buf_len = I2S_BUFFER_LEN;
    i2s_config.dma_buf_count = I2S_BUFFER_COUNT;
    i2s_config.channel_format = I2S_FORMAT;
    i2s_config.communication_format = I2S_COMM_FORMAT_I2S;

    //install and start i2s driver
    if (i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL) == ESP_OK)
        Serial.println("driver OK");
    if (i2s_set_pin(I2S_NUM, NULL) == ESP_OK)
        Serial.println("pin OK");
    if (i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL) == ESP_OK)
        Serial.println("mode OK");
    if (i2s_start(I2S_NUM) == ESP_OK)
        Serial.println("i2s OK");

    Serial.println("setup complete");
}

void setup()
{
    initSerial();
    initGpio();
    initWifi();
    initI2S();
}

void disp_buf(uint8_t *buf, int length)
{
#if I2S_BUF_DEBUG
    printf("======\n");
    for (int i = 0; i < length; i++)
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 4 == 0)
        {
            printf("\n");
        }
    }
    printf("======\n");
#endif
}

void send_buf(uint8_t *buf, int totalBytes)
{
    int numBits = I2S_SAMPLE_BITS;
    int numBytes = I2S_SAMPLE_BITS / 8;

    std::stringstream ss;
    for (int i = 0; i < totalBytes; i++)
    {
        for (int i = 0; i < numBytes; ++i)
            ss << std::hex << (int)buf[i];
        std::string str_hex = ss.str();

        // Serial.println(String(str_hex.c_str()));
        int i_hex = strtol(str_hex.c_str(), nullptr, 16);
        // Serial.println(String(i_hex));
        webSocketServer.sendData(String(i_hex));
        ss.str("");

        // uint32_t x = 0;
        // for (int j = 0; j < numBytes; j++)
        // {
        //     x |= (uint32_t)buf[i] << (numBits - ((j + 1) * 8));
        //     i++;
        // }
        // webSocketServer.sendData(String(x));
        // Serial.println(String(x));
    }
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
                int i2s_read_len = I2S_READ_LEN;
                char *i2s_read_buff = (char *)calloc(i2s_read_len, sizeof(char));
                i2s_adc_enable(I2S_NUM);

                int i = i2s_read_bytes(I2S_NUM, (char *)i2s_read_buff, (size_t)i2s_read_len, portMAX_DELAY);
                // disp_buf((uint8_t *)i2s_read_buff, i);
                send_buf((uint8_t *)i2s_read_buff, i);

                i2s_adc_disable(I2S_NUM);
                free(i2s_read_buff);
                i2s_read_buff = NULL;
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

            delay(10);
        }

        Serial.print("Disconnecting from: ");
        Serial.println(ip);
        delay(100);
    }
}
