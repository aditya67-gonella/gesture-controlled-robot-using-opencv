# gesture-controlled-robot-using-opencv
A robot which is controlled by gestures using OpenCV and Websockets.The connection between python and ESP8266 is successfully established using Websockets. Python is configured as server and Nodemcu(microcontroller) as client for successful data transfer.

#Flow of data:
-Initially the hand gesture is recognized using Opencv using processing techniques like masking, colour extraction etc <br> -Using contours the number of fingers opened can be calculated -Based on the number of fingers a key value will be tranferred to Nodemcu using websockets -Python program (Gesture recognition) acts as server and nodemcu as client for sending key value to Nodemcu -Upon on receiving through ESP8266 WiFi module Nodemcu actuates the motors using L298N Motor driver -Thus successful movement can be achieved
