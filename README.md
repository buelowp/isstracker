# Use an opaque globe to show the position of the ISS using a laser
This project is going to project the ISS location on a globe showing current position continuously. The system uses a Particle Argon, one stepper motor for longitude rotation, and one servo motor for latitude rotation. It also uses a LOT of other parts listed in the BOM to make this work.

The goal is to provide a good looking device that is accurate and consistent. All of the motors and electronics must be hidden, and all parts must be independently moveable. So, the globe must be able to rotate without changing where the laser is pointing. The laser must work inside the globe independent of the base. The laser must also rotate 360 degrees longitude. The ISS does not go above or below ~60 degrees latitude, so a simple servo can be used for that.

A Particle project named isstracker

I found a similar project using an ESP8266 about 5 years ago and I was intrigued. So, I started hacking something up, but my requirements became much more complex than the original project. I wanted full rotation and to be able to *spin* the globe sitting on a base without changing where the location of the point showing the ISS.

It uses two independent sliprings which provide the ability to rotate the globe and the armature holding the laser without losing precision. It uses one 200 step NEMA stepper for longitude, and one 160 degree servo for the latitiude. The laser is from Sparkfun, but any laser that can be powered by 3 volts would work.

Pictures to follow

## Building it

Instructions to follow