# X-Hulk (Brusheless Quad addon)

This add-on is useful when you want to fly a brusheless quadrotor using
X-Hulk addon. For more information on the addon please visit:
https://create.dronaaviation.com/hardware/accessories/introduction

## Supported Flight controllers
Primus X
Primus X2 (Under development)

## Connections
You can find the connections in the pdf link below:
X-Hulk Connections.png

### Code modifications
To enable Brushless drive, go to the respective target file(Primus X / Primus X2) and comment the #define BRUSHED_MOTORS. All motor PWMs run at 50Hz.
