# DualBLDCController
RP2040 Pico 2x Brushless motor controller! Uses SimpleFOC + Halls + TO-220 mosfets or IGBTs. Trying for low cost + lots of flexibility + solder it yourself at home + repairable. Works as a 3x DC motor controller as well.



**Features**

RP2040 Pico running Simplefoc (https://simplefoc.com/) and using halls (or -untested but should be possible- encoders) for motor commutation. (No FOC - RP2040 doesn't have enough A/D channels for 2x motor FOC.) Current sensing: 1 A/D channel per motor. Uses 2304 half-H drivers. Can assemble only half the board if you only want one servo amp. Spot for big diodes if you want 'em. Big motors seem to eat my mosfet body diodes rather quickly and IGBT's don't have body diodes. 

Control via usb serial or hardware serial. (Not both at same time that would be more complexity to break!) Human readable serial though, following the simplefoc commander schema: AE1 enables motor, A10 sets motor A to 10 rev/sec, A0 stops motor A. AE0 quickly drops power to motor a. etc... B10 sets motor B to 10 rev/sec... etc... This allows you to control, test, and modify this servo amp right from the Arduino IDE or pretty much anything with a USB port or serial port.

Boards have a spot for molex plugs that are pin compatible with Husqvarna 450x automower drive motors. (NOT the cut motor - it's a different pinout but you can re-pin the cut motor to match the drive motors and zoom off you go.) 

Board requires your motor power DC source, and ~12vdc@ say 50mA. The board can either make 3.3v for the RP2040 pico, or you can provide external 3.3v. 

There's a very simple heartbeat signal in the code - if the amp doesn't get an instruction from serial it'll shut down. 

**Assembly/Setup:**

Boards require SMD paste soldering and normal soldering to build: 

     SMD: It's not bad, order a solder mask when you order the board, place components, toaster oven for 3 mins. All SMD components are on the same side of the board. 
     
     Normal: in particular the mosfets. They are TO-220 because you are a hacker. You can solder in mosfets you have laying around and then de-solder them yourself to put better ones in. Or put in 3x0.1" pin headers so you can just plug in mosfets when you inevitably dead short them after trying to roll your own 3 phase driver. Basically you might want to 1. swap mosfets lots as you scratch write a 3phase driver. 2. use whatever mosfets are around. 3. repair it yourself. All of these would wreck a normal board but mean you can just break out your trusty soldering iron and fix this board instead of throwing it in the trash!

If you don't want to use the A/D (or want to repurpose any other inputs/outputs after you get a board you can cut the trace to that pin, and there's a tinned hole for you to take over.

'firmware' for the rp2040 has some basic configuration and setup instructions in comments at the top of the code.

**tested**

Tested using velocity control and torque control up to 10A/21v. Running in a Husqvarna 450x Automower as I write this. Maybe 10 hours of operation so far. Will update if/when anything fails.

Tested with 3x brushed (non 3 phase) motors using https://github.com/simplefoc/Arduino-FOC-dcmotor  - worked great but I did not test feedback just that it could spin the motors. Pretty sure there's only enough pins for 2x quadrature encoders.

**Thank you**

Big thank you to https://github.com/ClemensElflein/ for creating the Open Mower project and his very well thought out 3 phase rp2040 servo amp code. His project inspired me to take the leap and make this happen. This board can(at least used to!) run (a version of?)his servo amp code but you have to change your pins.h, and possibly more IDK. The original idea was use the openmower framework to control this servo amp but I ended up going a different direction.

Thanks to everyone who open sources their work - makes the world a better place. Hope my code helps you!

**Plans/Stuff that's broken, rough order:**

- Finish this documentation. Parts list looking at you.
- Current sense is at best not super calibrated, at worst needs work. Functions well enough to shut down the amp if current limit is hit.
- Test both DC and BLDC motors with an encoder instead of halls.
- Build more robots using this amp.
- Test it up to 240vdc/10A. Should work?
- Connect JUST the 2304's to a Mesa FPGA board running linuxCNC. Yep. dump the pico and put your own brain on this. 

**Next board iteration:**

-Heat sinks: Mosfet locations are not in a nice line. We've got 1 sink per mosfet, and once we start moving power we might want some real cooling here not just tiny heat sinks to air. 
-Fix the onboard 3.3v - Turns out you're not supposed to use linear regulators as DC-DC converters. Oops. It works but wastes 50mA. Leaning towards deleting it 'cause 3.3v or 5v can come from RPi or BBB or pretty much any other board you use to drive this. AND if you have external 3.3v you can dump power to the pico even if you leave the DC main power connected. Maybe a small fuse though?
-Make the power input and GND holes large enough to accommodate 12Ga wire.
-Fix various labels on the board

Cheers!
