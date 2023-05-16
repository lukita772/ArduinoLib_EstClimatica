## Weather station
This sensor use a rain detector stage, which detects rain and generate a signal that emulates a reset and blocks any innecesary noise.
The rain detector keeps the microcontroller alive, measuring the water by counts of a fixed quantity of water in a certain area.
When the rain stops, it send a message through wifi/mqtt with a set of climate-related data.

## Screenshots
![image](https://github.com/lukita772/ArduinoLib_EstClimatica/assets/117228370/f3330744-2db6-4987-8569-b525d8aab1ca)
