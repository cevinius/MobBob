The are 2 versions of the Arduino code here.

The "MobBob-Control-Bluno" version is designed for use with DFRobot's Bluno boards.
This is what I use in my MobBob builds as the Bluno board has Bluetooth LE built-in.

The "MobBob-Control-Bluetooth" version is designed for use with other Arduino
microcontrollers and Bluetooth boards. The Bluetooth board is connected to the
Arduino board using 2 digital pins (as a software serial port). Using this version
will let you use other Arduino boards (E.g. Arduino Nano, Arduino Pro) with other
Bluetooth boards (e.g. JY-MCU, HC-05, etc).

For both versions of the code, you will want to:

    - Update the pin variables to match your build
    
    - Tweak the servo center, min and max values
    
    - Set "FRONT_JOINT_HIPS" to 1 or -1 depending on which way your
      hip servos are mounted. I mount it with the servo axel at the front
      of MobBob. For this configuration, set this value to 1.

All the best! And have fun! :D