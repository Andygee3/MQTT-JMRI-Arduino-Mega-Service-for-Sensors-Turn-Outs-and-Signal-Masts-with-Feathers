# MQTT-JMRI-Arduino-Mega-Service-for-Sensors-Turn-Outs-and-Signal-Masts-with-Feathers
A project which setups up multiple Arduino Mega's to send and receive MQTT messages from JMRI to set Turnouts using slow motion turn outs,  send sensor feedback and set signals both heads via turn outs and signal masts.  The boards use JMRI Memory Variables to setup each board sending JSON objects for each item so only the board ID needs to be setup meaning changes to sensors, signals led brightness values, turn out throws can be managed by the JMRI head without the need to set the individual configs on each Arduino Board. 

Turn Outs and Signals use PCA9685 boards.

The original idea for this is based on Vintage80sModelRailway's setup but it has been adapted to meet my own needs.  

Memory Variable JSON Object Definitions:

Turn Out Setup - B10:T1	{"type":"turnout","turnout":"B10:T1", "setup": [{"B":"0","D":"0","P":"1","T":"0","Tv":"1500", "Cv":"2000"}]}

B = the board identity,  this is the only setup required on the individual Arduino Boards,  this is used so the board knows which items it should pay attention to
D = the PCA9685 board address,  you can chain multiple boards,  the config is setup currently for 2 boards but can be extended if more are needed,  this is used to identify which the item is connected to.
P = the PCA9685 pin the item is connected to
T = the turnout number - this should be 0 - 15 for each PCA9685 board setup, this is used to address the item in the class.
Tv = the PWM value when thrown
Cv = the PWM value when closed.

Sensor Setup - PD2:SEN1	{"type":"sensor","sensor":"PD2", "setup": [{"B":"0","P":"31","I":"0","S":"9"}]}
Same as above +:
I = Invert the sensor
S = the sensor number 0 - 32 used to address the sensor in the in class

Signal Head Setup - SH10	{"type":"signal","head":"B9:S1:F", "setup": [{"B":"0","D":"0","P":"4","S":"9","Tv":"0","Cv":"4096"}]}
Same as above +:
S = Signal Head No used to address the item in the Signal Head Class
Tv = PWM value when off
Cv = PWM value when on (allows you to change the brightness per led)

Signal Mast Setup - SM1	{"type":"signalMast","mast":"B9|S1","setup":[{"B":"0","D":"0","Pr":"15","Ca":"14","Cp":"13","Da":"12","Ax1":"11","SM":"0","Ov":"4096","Lv":"1095","PrLv":"300","CaLv":"2600","CpLv":"2600","DaLv":"3500" ,"Ax1Lv":"1095"}]}

Same as above +:

Pr: PCA Pin for the Proceed Aspect
Ca: PCA Pin for the Caution Aspect (will flash if Flash Caution is set)
Cp: PCA Pin for the additional Preliminary Caution Aspect (will flash is Flash Preliminary Caution is set)
Da: PCA Pin for the Danger Aspect
Ax1-6:  The Pin used for the Feathers from One to Six
Ov: Off PWM value
Lv:  Lit PMW Value Default
?Lv:  i.e. PrLv:  use the Aspect code + Lv to set a PWM value for the individual aspects, i.e. "PrLv":"1026" would set the Proceed Led to approx a quarter of the light value of the Defaults Ov: 4096.  This allows you to override the default value for some or all of the aspects.

