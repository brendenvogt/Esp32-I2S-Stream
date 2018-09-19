# Esp32-I2S-Stream
A C++/Arduino project for the Esp32. Streaming data through a websocket over wifi network hosted by an Esp32.

This program opens a new **wifi access point** called **Esp32** with password **testpass** <br/>
It then sets up various things such as the GPIO pin 2 to turn on and off the Esp32 led. <br/>

The main loop continuously checks for incoming network connections on the access point. When a device is detected, it begins to try to establish a websocket handshake. If the it is successful, will then try to send an incrementing integer value over the socket if we have streaming enabled. <br/>
Sending the command **start** will begin streaming a continuously incrementing integer over the websocket connection, as well as turn on the onboard LED. <br/> 
Sending the command **stop** will stop the counter and stop the counter and stop streaming, as well as turn off the onboard LED. <br/>
Sending the command **reset** will reset the counter back to 0.

<!-- <img src="https://github.com/brendenvogt/Esp32-Websocket-Stream/raw/master/resources/Esp32Stream.gif"/> -->
