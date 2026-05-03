
#align(center)[ #title("
Intelligent Wristband Based on ESP32
 Project Oveview") ]

#align(center)[Team Leader: Moneer Mohammed]

// #set text(size: 1.15em)

#figure(
  image("project_overview.jpg", width: 80%),
  caption: [
    Components of the prototype.
  ],
)

The project monitors mainly pulse rate and burned calories using two sensors: A MAX30102 Heart Rate sensor through photoplethysmography (PPG), which detects changes in blood volume by measuring light absorption in skin, interfaced through I2C, and an MPU-6050/ADXL345 gyroscope/accelerometer that functions as a pedometer (step counter) through evaluating the received angular accelerations as movement (steps) and calculating the equivalent caloric consumption.

#columns(2)[
  Sensors are controlled via the ESP32-C3 Super Mini for its small form-factor, power consumption, and built-in WiFi module powered using a 3.7V rechargeable Li-ion battery regulated using a TP4056 regulator.

  Considering future smart-home applications, the processed data is sent using the MQTT protocol due to its publish/subscribe model. A Node-Red dashboard server is used to listen on the MQTT channel and trigger warnings/log diagnostics based on the received vital data.
  #colbreak()
  #figure(
    box(width: 9cm, height: 5cm, clip: true, place(dx: 1cm, dy: 0.1em, image("esp-mini.png", width: 100%))),
    caption: [
      The ESP32-C3 Super Mini Board.
    ],
  )
]

The prototype will consist of a small housing on a wristband holding the battery and a small PCB with the MCU and sensors. The possibility of removing the ESPRESSIF-based chip and routing it directly on the PCB will be investigated after having a working V1 on a perfboard.
