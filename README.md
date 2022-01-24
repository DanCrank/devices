# rfm69-sx1231-rpi

This began as a fork of github.com/tve/devices to try to make it work with
the current version of periph.io. This was a result of my looking for a
working library for the Adafruit RFM69 Raspberry Pi bonnet. Over the
course of making it work, I've ended up stripping out everything from the
old tve/devices module except the rfm69 / sx1231 bits, and simplifying the
directory structure.

This is being written specifically to the Adafruit RFM69 bonnet, but it
may work (or at least be adaptable) in other applications.

Adafruit guide for the RFM69 bonnet:
https://learn.adafruit.com/adafruit-radio-bonnets

Datasheet for the rfm69 / sx1231 module:
https://cdn-shop.adafruit.com/product-files/3076/sx1231.pdf