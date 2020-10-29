# Singing Jack-o-Lantern for Arduino

Moves 3 singing jack-o-lanterns and one skull with servo driven jaws. All the pumpkins have RGB eyes. One pumpkin also has retractable vampire fangs. 
The skull has a retractable trumpet with a cartoon-style animated bell. Also supports sending a UART signal to a second microcontroller as a switch. 
In this case, the second controller drives a small LCD screen, and the UART signal selects the image to display from an attached SD card, affecting
a crude animation. Also supports 53 (though the number is arbitrary) individually addressable RGB LEDs, controlled by WS2812b controllers. A large
number of digital channels are also supported. I am currently using 2 to drive solid state relays (SSRs) to switch mains current to light strands.
Finally, also reads a microphone peak detector, driving a separate skull based on audio level, which is attached to a amplified microphone,
allowing the user to speak through the skull from a safe distance.

  - 3 jack-o-lanterns with moving jaws and RGB eyes and one set of retractable fangs
  - 1 skull with moving jaws a trumpet with an animated arm and an animated bell
  - 53 individually addressable RGB LEDs
  - 2 solid state relay controlled light strands
  - 1 UART controlled LCD screen
  - 1 microphone controlled skull
  
  For 179 total animated channels.
