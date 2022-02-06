# NUCLEO-F401RE Demo for SSD1306 Driver
 


## Follow the project on [Hackaday](https://hackaday.io/project/181543-no-buffer-ssd1306-display-driver-for-stm32)

Some projects require a compact and straightforward human interface to debug/monitor the board status, like displaying the IP address, a connection status, or functional parameters. The OLED SSD1306 is an inexpensive solution for this kind of need.

I tried the Adfruit library in the past, but it is pretty bulky; other drivers draw the fonts pixel by pixel, and they all require a shadow memory buffer inside the microcontroller; In this specific case, even if the display is tiny, the buffer still uses 1KB. 

Here is an excellent solution using only 30 Bytes of your MCU RAM.
The driver can draw four different character sizes: a tiny 5x8, if you need to display a lot of info, a 10x16, which is very readable but still compact, and 15x24 and a 20x32, ideal for clock application.

The demo project uses an STM board NUCLEO F401RE, but it can be easly adapted to different Boards/MCU.