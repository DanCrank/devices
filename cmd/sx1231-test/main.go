// Copyright (c) 2016 by Thorsten von Eicken, see LICENSE file for details

package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"time"

	"github.com/DanCrank/devices/spimux"
	rfm69 "github.com/DanCrank/devices/sx1231"
	"periph.io/x/periph/conn/gpio/gpioreg"
	"periph.io/x/periph/conn/spi/spireg"
	"periph.io/x/periph/host"
)

func run(intrPinName, csPinName string, csVal, power int, debug bool) error {
	if _, err := host.Init(); err != nil {
		return err
	}

	intrPin := gpioreg.ByName(intrPinName)
	if intrPin == nil {
		return fmt.Errorf("cannot open pin %s", intrPinName)
	}

	selPin := gpioreg.ByName(csPinName)
	if selPin == nil {
		return fmt.Errorf("cannot open pin %s", csPinName)
	}

	spiBus, err := spireg.Open("")
	if err != nil {
		return err
	}

	spi1231, b := spimux.New(spiBus, selPin)
	if csVal != 0 {
		spi1231 = b
	}

	log.Printf("Initializing sx1231...")
	t0 := time.Now()
	rfm69, err := rfm69.New(spi1231.Port, intrPin, rfm69.RadioOpts{
		Sync:   []byte{0x2D, 0x06},
		Freq:   912500000,
		Rate:   49230,
		Logger: log.Printf,
	})
	if err != nil {
		return err
	}
	log.Printf("Ready (%.1fms)", time.Since(t0).Seconds()*1000)

	if len(os.Args) > 1 && os.Args[1] == "tx" {

		// need a receiving goroutine to keep things moving...
		go func() {
			for {
				rfm69.Receive()
			}
		}()

		for i := 1; i <= 20; i++ {
			log.Printf("Sending packet %d ...", i)
			t0 = time.Now()
			if i&1 == 0 {
				rfm69.SetPower(0x18)
			} else {
				rfm69.SetPower(0x1F)
			}
			//msg := "\x01Hello there, these are 60 chars............................"
			msg := []byte(fmt.Sprintf("\x01Hello %03d", i))
			if err := rfm69.Transmit(msg); err != nil {
				return err
			}
			time.Sleep(100 * time.Millisecond)
		}

		time.Sleep(100 * time.Millisecond)
		log.Printf("Bye...")

	} else {

		log.Printf("Receiving packets ...")
		for {
			pkt, err := rfm69.Receive()
			if err != nil {
				return err
			}
			log.Printf("Got len=%d rssi=%ddB fei=%dHz %q",
				len(pkt.Payload), pkt.Rssi, pkt.Fei, string(pkt.Payload))
		}
	}
	return nil
}

func main() {
	intrPin := flag.String("intr", "XIO-P0", "sx1231 radio interrupt pin name")
	csPin := flag.String("cspin", "CSID0", "sx1231 radio chip select pin name")
	csVal := flag.Int("csval", 0, "sx1231 radio chip select value (0 or 1)")
	power := flag.Int("power", 15, "sx1231 radio output power in dBm (2..17)")
	debug := flag.Bool("debug", false, "enable debug output")
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "Usage: %s:\n", os.Args[0])
		flag.PrintDefaults()
		os.Exit(1)
	}
	flag.Parse()

	if err := run(*intrPin, *csPin, *csVal, *power, *debug); err != nil {
		fmt.Fprintf(os.Stderr, "Exiting due to error: %s\n", err)
		os.Exit(2)
	}
}
