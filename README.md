# TechZone Communications Huxley
Firmware build for the old TechZone Communications Huxley RepRap

The initial contents of this repo come from here, or from links found there:
https://reprap.org/wiki/TechZoneHuxley

I found that the last firmware left at reprap.org for posterity had some details, so the first thing to do is obviously put it under version control, then implement fixes.

# History
I bought this printer kit something like 10 years ago. To receive it at an affordable price (i.e.: not pay a ridiculous amount for international S&H etc), I had to impose on a new colleague. Back then, new hires were sent on a training trip for a month, and as part of the "initiation", they are "required" to be the "mule" for the senior colleagues, i.e.: bring back stuff we had bought. So this new guy got the short straw with me, and had to come home with a bags full of screws, nuts, steppers, electronics, etc :)
I assembled it, calibrated most of the settings, did exactly 2 test prints, and had only 1 setting left to calibrate: the amount of filling (the roof of a cube ended up sagging inwards), when life intruded, and I got married. 

I gave it a few tries shortly after, but the Y axis started acting weird, and I could never get the straight of why. Then, the revolution happened, and today we're flooded with brand printers at low prices.

10 years after purchasing, and I finally have the opportunity to bring it back to life. I found that the problem was that the Y axis cog was cracked. I bought a cheap Ender 3, found the stl designs on reprap.org, printed a new one, and done: it works.

I also figured out how (again) how to build the original firmware, with the Arduino core from back then, with the Arduino IDE from back then. So I can meddle with it again.

Given the adventure, I don't want to get rid of it.

I _could_ do away with the eletronics and buy a ramps. I _could_ do away with the tip/bed managers and figure out how to connect them the way things are done today. I _could_ do away with the entire printer and just stick with my new Ender. But what would be the fun in that?

# Contribution Policy
This is primarily a personal project. 

If you want to contribute, sure, I'll take a look at your PR. 

If you want to discuss and work together, sure, open an issue. 

If you want me to use time to help you, you're in the wrong place.

# License
According to [reprap.org](reprap.org), the [designs and firmware](https://reprap.org/wiki/TechZoneHuxley) are GPL.

According to the [Sanguino project on the Google Code Archive](https://code.google.com/archive/p/sanguino/downloads), the core is also GPL.

I _think_ the old Arduino IDE is also GPL, per the github page [here](https://github.com/arduino/Arduino/blob/0021/license.txt).

# Acknowledgements
Thanks to @moose4621 for finding the info again at reprap.org, I had long lost the links. And for getting me back into XYZ machines!

Thanks to TechZone Communications and the engineers who published the details for posterity. I consider the info left behind an example of future proofing a product, and it is certainly an example in open source. 10 years later, and I am able to print replacement parts, tweak and rebuild the firmware, follow the assembly instructions, and overall figure things out, all thanks things having been done right. I hope somebody who was a part of all of that reads this some day.
