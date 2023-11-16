# ratgdo - WW version

This version is radically different from the upstream version:
1. written entirely in C using esp-idf - only supports ESP32 for now (and probably forever)
1. OTA updates
   1. integrated into the WW management system
1. only support security+ 2 for now
1. dry contact control has been removed (expectation is buttons would be separate IoT devices)
1. includes minimal CI/CD and full build environment
1. circuit diagrams are here: https://github.com/Kaldek/rat-ratgdo/tree/main

# basically, it's just for me but if somebody finds it helpful to reference, go for it
* yes, the components are private for now so if you want to build it yourself drop me a line on mastodon.

# Original README content
ratgdo gives you **local** MQTT & dry contact control plus status feedback for your residential Chamberlain/LiftMaster/Merlin garage door opener _including_ Security+ 2.0 openers. (Security+ 2.0 uses an encrypted serial signal to control the door opener's open/close and light functions, which makes it impossible to use Shelly One, Go Control, Insteon I/O linc or any other dry contact relay device to control the door). 

# [Visit the github.io page for instructions](https://paulwieland.github.io/ratgdo/).
[ratgdo on GitHub.io](https://paulwieland.github.io/ratgdo/)

# Special thanks

Special thanks to Paul Wieland, kosibar, Brad and TechJunkie01 - without whom this project would not have been possible.