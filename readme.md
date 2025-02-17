# joybus-pio rs

An implementation of the controller side of the joybus protocol for gamecube for the RP2040 chip via its PIO functionality.

[joybus-pio](https://github.com/JonnyHaystack/joybus-pio) and [haybox](https://github.com/JonnyHaystack/HayBox) were heavily referenced in the implementation.

For an example usage see [rukaibox_firmware](https://github.com/rukai/rukaibox_firmware)

## Goals

### Currently implemented

* Supports gamecube (joybus) controller protocol.

### Things I would be happy for others to implement

* N64 support

### Things I might be happy for others to implement

* Console side protocol support

## Non-Goals

* Support for anything other than rp2040 PIO
  * If boards start using the RP2350 chip, or the RP2040 is in some other way seriously outdated I will consider moving to a new chip.
