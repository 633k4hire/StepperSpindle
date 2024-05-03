 #pragma once

 //seeduino xiao esp32c3 https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/
 // GPIO pin configuration for inputs
    #define  dirPinIn 20       // Direction input pin
    #define enablePinIn 8    // Enable input pin
    #define stepPinIn 9      // Step input pin
    #define pwmInPin 10       // PWM input pin for spindle speed control

    // GPIO pin configuration for outputs
    #define dirPinOut 21    // Direction output pin to stepper
    #define enablePinOut 7   // Enable output pin to stepper
    #define stepPinOut 6     // Step output pin to stepper
