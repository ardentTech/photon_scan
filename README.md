# Photon Scan
TODO STM32F767ZI, FreeRTOS

## System
### Servos
- The two Tower SG90 servos driving the pan-tilt mechanism rely on two TIM3 channels for a consistent 50Hz PWM signal.
- When activated, the servos can follow a multi-step seqeuence to traverse their available range of travel.
- The servo sequence is managed by a state machine and a static array of predefined movements.
- At each step of the sequence, the four ADC channels are sampled.
- Per sequence, min and max ADC values are determined as well as respective servo positions.

### LDR Quad ###
- Four channels of ADC1 are used to read voltages from the 5516 LDRs and then write the values into a circular buffer on DMA2.
- A custom bracket for the four 5516 LDRs was designed and 3d-printed to fit into the top of the pan-tilt mechanism.

### User Inputs ###
- A user can press the on-board blue button to start the servo sequence.
- While the servo sequence is running, a user can press the on-board blue button to cancel the sequence.

## TODO
- [ ] port to clion
- [ ] integrate cmake
- [ ] integrate unity
- [ ] unit tests
- [ ] replace LDRs (in-progress)
- [ ] circuit diagram
- [ ] scanner state machine diagram
- [ ] integrate rotary encoders
- [ ] real-time tracking
- [ ] servo easing