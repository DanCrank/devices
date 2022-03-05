// Copyright 2016 by Thorsten von Eicken, see LICENSE file
// Modified 2022 by Dan Crank, danno@danno.org

// The SX1231 package interfaces with an AdaFruit RFM69 radio connected to an SPI bus.
//
// This package is a significant simplification over the codebase it was originally
// forked from. It is no longer interrupt driven: the Receive() and Transmit()
// functions block until complete.

package sx1231

import (
	"errors"
	"fmt"
	"log"
	"sync"
	"time"

	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
)

// Radio represents a Semtech SX1231 radio as used in HopeRF's RFM69 modules.
type Radio struct {
	// configuration
	spi     spi.Conn   // SPI device to access the radio
	intrPin gpio.PinIn // interrupt pin for RX and TX interrupts
	sync    []byte     // sync bytes
	freq    uint32     // center frequency
	rate    uint32     // bit rate from table
	paBoost bool       // true: use PA1+PA2 power amp, else PA0
	power   byte       // output power in dBm
	// state
	sync.Mutex           // guard concurrent access to the radio
	mode       byte      // current operation mode
	log        LogPrintf // function to use for logging
}

// RadioOpts contains options used when initilizing a Radio.
type RadioOpts struct {
	Sync    []byte    // RF sync bytes
	Freq    uint32    // frequency in Hz, Khz, or Mhz
	Rate    uint32    // data bitrate in bits per second, must exist in Rates table
	PABoost bool      // true: use PA1+PA2, false: use PA0
	Logger  LogPrintf // function to use for logging
}

// Rate describes the SX1231 configuration to achieve a specific bit rate.
//
// The datasheet is somewhat confused and confusing about what Fdev and RxBw really mean.
// Fdev is defined as the deviation between the center freq and the modulated freq, while
// conventionally the frequency deviation fdev is the difference between the 0 and 1 freq's,
// thus the conventional fdev is Fdev*2.
//
// Similarly the RxBw is specified as the single-sided bandwidth while conventionally the
// signal or channel bandwidths are defined using the total bandwidths.
//
// Given that the sx1231 is a zero-if receiver it is recommended to configure a modulation index
// greater than 1.2, e.g. best approx 2. Modulation index is defined as fdev/bit-rate. This
// means that Fdev should be approx equal to bps. [Martyn, or are you targeting a modulation
// index of 4?]
//
// The signal bandwidth (20dB roll-off) can be approximated by fdev + bit-rate. Since RxBw
// is specified as the single-sided bandwidth it needs to be at least (fdev+bit-rate)/2. Or,
// in sx1231 config terms, Fdev + bitrate/2. If AFC is used, in order to accommodate a crystal
// offset between Tx and Rx of Fdelta the AFC bandwidth should be approx fdev + bit-rate +
// 2*Fdelta.
type Rate struct {
	Fdev    int  // TX frequency deviation in Hz
	Shaping byte // 0:none, 1:gaussian BT=1, 2:gaussian BT=0.5, 3:gaussian BT=0.3
	RxBw    byte // value for rxBw register (0x19)
	AfcBw   byte // value for afcBw register (0x1A)
}

// Rates is the table of supported bit rates and their corresponding register settings. The map
// key is the bit rate in bits per second. In order to operate at a new bit rate the table can be
// extended by the client.
var Rates = map[uint32]Rate{
	9600:  {19013, 0, 0xF4, 0xF4},  // DanCrank setting for rover project
	49230: {45000, 0, 0x4A, 0x42},  // JeeLabs driver for rfm69 (RxBw=100, AfcBw=125)
	49231: {180000, 0, 0x49, 0x49}, // JeeLabs driver with rf12b compatibility
	49232: {45000, 0, 0x52, 0x4A},  // JeeLabs driver for rfm69 (RxBw=83, AfcBw=100)
	49233: {51660, 0, 0x52, 0x4A},  // JeeLabs driver for rfm69 (RxBw=83, AfcBw=100)
	50000: {90000, 0, 0x42, 0x42},  // nice round number
}

// RxPacket is a received packet with stats.
type RxPacket struct {
	Payload []byte    // payload, from address to last data byte, excluding length & crc
	Rssi    int       // rssi value for current packet
	Snr     int       // rssi - noise floor for current packet
	Fei     int       // frequency error for current packet
	At      time.Time // time of rx interrupt
}

// Temporary is an interface implemented by errors that are temporary and thus worth retrying.
type Temporary interface {
	Temporary() bool
}

//const hwDelay time.Duration = 100 * time.Millisecond //generic delay to allow hardware to cope with non-root access
//see: https://forum.up-community.org/discussion/2141/solved-tutorial-gpio-i2c-spi-access-without-root-permissions

// New initializes an sx1231 Radio given an spi.Conn and an interrupt pin, and places the radio
// in receive mode.
//
// The SPI bus must be set to 10Mhz max and mode 0.
//
// The RF sync bytes used are specified using the sync array, the frequency is specified
// in Hz, Khz, or Mhz, and the data bitrate is specified in bits per second and must match one
// of the rates in the Rates table.

func New(port spi.Port, cs gpio.PinOut, reset gpio.PinOut, intr gpio.PinIn, opts RadioOpts) (*Radio, error) {
	r := &Radio{
		intrPin: intr,
		mode:    255,
		paBoost: opts.PABoost,
		log:     func(format string, v ...interface{}) {},
	}
	if opts.Logger != nil {
		r.log = func(format string, v ...interface{}) {
			opts.Logger("sx1231: "+format, v...)
		}
	}

	// Set SPI parameters and get a connection.
	conn, err := port.Connect(2*physic.MegaHertz, spi.Mode0, 8)
	if err != nil {
		return nil, fmt.Errorf("sx1231: cannot set device params: %v", err)
	}
	r.spi = conn

	// this reset sequence may be peculiar to the Adafruit RFM69 bonnet
	// this mimics the same thing their CircuitPython code does
	reset.Out(gpio.High)
	time.Sleep(100 * time.Millisecond)
	reset.Out(gpio.Low)
	time.Sleep(1 * time.Second)

	r.setMode(MODE_SLEEP)
	r.setMode(MODE_STANDBY)

	// Detect chip version.
	r.log("SX1231/SX1231 version %#x", r.readReg(REG_VERSION))

	// Write the configuration into the registers.
	for i := 0; i < len(configRegs)-1; i += 2 {
		r.writeReg(configRegs[i], configRegs[i+1])
	}
	r.setMode(MODE_STANDBY)
	// Debug: read config regs back and make sure they are correct.
	for i := 0; i < len(configRegs)-1; i += 2 {
		if r.readReg(configRegs[i]) != configRegs[i+1] {
			r.log("error writing config reg %#x: got %#x expected %#x",
				configRegs[i], r.readReg(configRegs[i]), configRegs[i+1])
		}
	}

	// Configure the bit rate and frequency.
	r.SetRate(opts.Rate)
	r.SetFrequency(opts.Freq)
	r.SetPower(17)

	// Configure the sync bytes.
	if len(opts.Sync) < 1 || len(opts.Sync) > 8 {
		return nil, fmt.Errorf("sx1231: invalid number of sync bytes: %d, must be 1..8",
			len(r.sync))
	}
	r.sync = opts.Sync
	wBuf := make([]byte, len(r.sync)+2)
	rBuf := make([]byte, len(r.sync)+2)
	wBuf[0] = REG_SYNCCONFIG | 0x80
	wBuf[1] = byte(0x80 + ((len(r.sync) - 1) << 3))
	copy(wBuf[2:], r.sync)
	r.spi.Tx(wBuf, rBuf)

	// log register contents
	r.logRegs()

	// Finally turn on the receiver.
	r.setMode(MODE_RECEIVE)

	return r, nil
}

// SetFrequency changes the center frequency at which the radio transmits and receives. The
// frequency can be specified at any scale (hz, khz, mhz). The frequency value is not checked
// and invalid values will simply cause the radio not to work particularly well.
func (r *Radio) SetFrequency(freq uint32) {
	r.Lock()
	defer r.Unlock()

	// accept any frequency scale as input, including KHz and MHz
	// multiply by 10 until freq >= 100 MHz
	for freq > 0 && freq < 100000000 {
		freq = freq * 10
	}
	r.freq = freq
	r.log("SetFrequency: %dHz", freq)

	mode := r.mode
	r.setMode(MODE_STANDBY)
	// Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
	// use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
	// due to this, the lower 6 bits of the calculated factor will always be 0
	// this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
	// 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000
	frf := (freq << 2) / (32000000 >> 11)
	//log.Printf("SetFreq: %d %d %02x %02x %02x", freq, uint32(frf<<6),
	//	byte(frf>>10), byte(frf>>2), byte(frf<<6))
	r.writeReg(REG_FRFMSB, byte(frf>>10), byte(frf>>2), byte(frf<<6))
	r.setMode(mode)
}

// SetRate sets the bit rate according to the Rates table. The rate requested must use one of
// the values from the Rates table. If it is not, nothing is changed.
func (r *Radio) SetRate(rate uint32) {
	params, found := Rates[rate]
	if !found {
		return
	}
	bw := func(v byte) int {
		return 32000000 / (int(16+(v&0x18>>1)) * (1 << ((v & 0x7) + 2)))
	}
	r.log("SetRate %dbps, Fdev:%dHz, RxBw:%dHz(%#x), AfcBw:%dHz(%#x) AFC off:%dHz", rate,
		params.Fdev, bw(params.RxBw), params.RxBw, bw(params.AfcBw), params.AfcBw,
		(params.Fdev/10/488)*488)

	r.Lock()
	defer r.Unlock()

	r.rate = rate
	mode := r.mode
	r.setMode(MODE_STANDBY)
	// program bit rate, assume a 32Mhz osc
	var rateVal uint32 = (32000000 + rate/2) / rate
	r.writeReg(REG_BITRATEMSB, byte(rateVal>>8), byte(rateVal&0xff))
	// program frequency deviation
	var fStep float64 = 32000000.0 / 524288 // 32Mhz osc / 2^19 = 61.03515625 Hz
	fdevVal := uint32((float64(params.Fdev) + fStep/2) / fStep)
	r.writeReg(REG_FDEVMSB, byte(fdevVal>>8), byte(fdevVal&0xFF))
	// program data modulation register
	r.writeReg(REG_DATAMODUL, params.Shaping&0x3)
	// program RX bandwidth and AFC bandwidth
	r.writeReg(REG_RXBW, params.RxBw, params.AfcBw)
	// program AFC offset to be 10% of Fdev
	r.writeReg(REG_TESTAFC, byte(params.Fdev/10/488))
	if r.readReg(REG_AFCCTRL) != 0x00 {
		r.setMode(MODE_FS)            // required to write REG_AFCCTRL, undocumented
		r.writeReg(REG_AFCCTRL, 0x00) // 0->AFC, 20->AFC w/low-beta offset
	}
	r.setMode(mode)
}

// SetPower configures the radio for the specified output power (TODO: should be in dBm).
func (r *Radio) SetPower(dbm byte) {
	r.Lock()
	defer r.Unlock()

	// Save current mode.
	mode := r.mode
	r.setMode(MODE_STANDBY)

	if r.paBoost {
		// rfm69H with external antenna switch.
		if dbm > 20 {
			dbm = 20
		}
		switch {
		case dbm <= 13:
			r.writeReg(REG_PALEVEL, 0x40+18+dbm) // PA1
		case dbm <= 17:
			r.writeReg(REG_PALEVEL, 0x60+14+dbm) // PA1+PA2
		default:
			r.writeReg(REG_PALEVEL, 0x60+11+dbm) // PA1+PA2+HIGH_POWER
		}
	} else {
		// rfm69 without external antenna switch.
		if dbm > 13 {
			dbm = 13
		}
		r.writeReg(REG_PALEVEL, 0x80+18+dbm) // PA0
	}
	// Technically the following two lines are for <=17dBm, but if we're set higher
	// then the registers get writte with the correct value each time the mode is switched
	// into Tx or Rx, so it's safe to do it unconditionally here.
	r.writeReg(REG_TESTPA1, 0x55)
	r.writeReg(REG_TESTPA2, 0x70)
	r.log("SetPower %ddBm", dbm)
	r.power = dbm

	// Restore operating mode.
	r.setMode(mode)
}

// SetEncryptionKey configures the radio to use encryption with the specified key
func (r *Radio) SetEncryptionKey(key []byte) {
	if len(key) != 16 {
		log.Fatalf("invalid key length specified for SetEncryptionKey (expected 16 bytes, got %d)", len(key))
	}

	r.Lock()
	defer r.Unlock()

	// Save current mode.
	mode := r.mode
	r.setMode(MODE_STANDBY)

	pktCfg2 := r.readReg(REG_PKTCONFIG2)
	r.writeReg(REG_PKTCONFIG2, pktCfg2|0x01) // turn on AES encryption
	r.writeReg(REG_AESKEYMSB, key...)

	// Restore operating mode.
	r.setMode(mode)
}

// SetEncryptionOff configures the radio to not use encryption
func (r *Radio) SetEncryptionOff() {
	r.Lock()
	defer r.Unlock()

	// Save current mode.
	mode := r.mode
	r.setMode(MODE_STANDBY)

	pktCfg2 := r.readReg(REG_PKTCONFIG2)
	r.writeReg(REG_PKTCONFIG2, pktCfg2&0xFE) // turn off AES encryption

	// Restore operating mode.
	r.setMode(mode)
}

// LogPrintf is a function used by the driver to print logging info.
type LogPrintf func(format string, v ...interface{})

// setMode changes the radio's operating mode and changes the interrupt cause (if necessary), and
// then waits for the new mode to be reached.
func (r *Radio) setMode(mode byte) {
	mode = mode & 0x1c
	switch mode {
	case MODE_TRANSMIT:
		r.log("setMode: MODE_TRANSMIT")
	case MODE_RECEIVE:
		r.log("setMode: MODE_RECEIVE")
	case MODE_STANDBY:
		r.log("setMode: MODE_STANDBY")
	case MODE_SLEEP:
		r.log("setMode: MODE_SLEEP")
	case MODE_FS:
		r.log("setMode: MODE_FS")
	default:
		r.log("setMode: MODE_??? %d", mode)
	}

	// If we're in the right mode then don't do anything.
	if r.mode == mode {
		r.log("setMode: no-op")
		return
	}

	// Set the interrupt mode if necessary.
	switch mode {
	case MODE_TRANSMIT:
		if r.power > 17 {
			// To get >17dBm on the rfm69H some magic is required.
			r.writeReg(REG_TESTPA1, 0x5D)
			r.writeReg(REG_TESTPA2, 0x7C)
		}
		// Setting DIO_PKTSENT will not cause an intr.
		r.writeReg(REG_DIOMAPPING1, DIO_MAPPING+DIO_PKTSENT)
		// Set the new mode.
		r.writeReg(REG_OPMODE, mode)
	case MODE_RECEIVE:
		if r.power > 17 {
			// Turn off high power Tx stuff.
			r.writeReg(REG_TESTPA1, 0x5D)
			r.writeReg(REG_TESTPA2, 0x7C)
		}
		// We get here from MODE_FS and DIO_MAPPING, we need to switch to RX first and then
		// change DIO.
		r.writeReg(REG_OPMODE, mode)
		r.writeReg(REG_DIOMAPPING1, DIO_MAPPING+DIO_RSSI) // DIO_RSSI or DIO_SYNC
	default:
		// Mode used when switching, make sure we don't get an interupt.
		if r.mode == MODE_RECEIVE {
			r.writeReg(REG_DIOMAPPING1, DIO_MAPPING)
			r.writeReg(REG_OPMODE, mode)
		} else {
			r.writeReg(REG_OPMODE, mode)
			r.writeReg(REG_DIOMAPPING1, DIO_MAPPING)
		}
	}

	// Busy-wait 'til the new mode is reached.
	for start := time.Now(); time.Since(start) < 100*time.Millisecond; {
		if val := r.readReg(REG_IRQFLAGS1); val&IRQ1_MODEREADY != 0 {
			r.mode = mode
			return
		}
	}
	//r.err = errors.New("sx1231: timeout switching modes")
}

// blocking (non-interrupt driven) receive
func (r *Radio) Receive(timeout time.Duration) (*RxPacket, error) {
	//r.log("In Receive")
	r.logRegs()
	r.setMode(MODE_RECEIVE)
	// wait for a packet ready, up to the timeout
	payloadReady := false
	end := time.Now().Add(timeout)
	for !time.Now().After(end) {
		payloadReady = r.readReg(REG_IRQFLAGS2)&IRQ2_PAYLOADREADY != 0
		if payloadReady {
			break
		}
		time.Sleep(100 * time.Millisecond)
	}
	// if timeout, return nil
	if !payloadReady {
		//r.log("Receive: timeout, returning nil")
		r.logRegs()
		return nil, nil
	}
	//r.log("Receive: radio reports payload ready")
	r.logRegs()
	// set standby mode
	r.setMode(MODE_STANDBY)
	// read packet from fifo
	var rssi, fei int
	// Bail out if we're not actually receiving a packet. This happens when the
	// receiver restarts because RSSI went away or no SYNC was found before timeout.
	irq1 := r.readReg(REG_IRQFLAGS1)
	// As soon as we have sync match, grab RSSI and FEI.
	if rssi == 0 && irq1&IRQ1_SYNCMATCH != 0 {
		// Get RSSI.
		rssi = 0 - int(r.readReg(REG_RSSIVALUE))/2
		// Get freq error detected, caution: signed 16-bit value.
		f := int(int16(r.readReg16(REG_AFCMSB)))
		fei = (f * (32000000 >> 13)) >> 6
	}
	var wBuf, rBuf [67]byte
	wBuf[0] = REG_FIFO
	//r.log("Receive: reading fifo")
	r.spi.Tx(wBuf[:], rBuf[:])
	buf := rBuf[1:] // ?
	// build and return packet
	l := buf[0]
	if l > 65 {
		r.log("Rx packet too long (%d)", l)
		return nil, fmt.Errorf("received packet too long (%d)", l)
	}
	var snr int
	if rssi != 0 {
		floor := -int(r.readReg(REG_RSSITHRES)) / 2
		snr = rssi - floor
		r.log("RX Rssi=%d Floor=%d SNR=%d", rssi, floor, snr)
	}
	r.log("Receive: returning packet")
	return &RxPacket{Payload: buf[1 : 1+l], Rssi: rssi, Snr: snr, Fei: fei}, nil
}

// blocking (non-interrupt driven) send
func (r *Radio) Transmit(payload []byte) error {
	r.Lock()
	defer r.Unlock()

	// limit the payload to valid lengths
	switch {
	case len(payload) > 65:
		payload = payload[:65]
	case len(payload) == 0:
		return errors.New("invalid payload length")
	}
	r.setMode(MODE_FS)

	// push the message into the FIFO.
	buf := make([]byte, len(payload)+1)
	buf[0] = byte(len(payload))
	copy(buf[1:], payload)
	r.writeReg(REG_FIFO|0x80, buf...)
	r.log("Transmit(): message loaded")
	r.logRegs()
	r.setMode(MODE_TRANSMIT)
	//changing this to a blocking send - wait here until the hardware reports that the send is complete
	//TODO: put some kind of a timeout on this block
	r.log("Transmit(): transmit mode set, waitng for transmit complete")
	for {
		if irq2 := r.readReg(REG_IRQFLAGS2); irq2&IRQ2_PACKETSENT == 0 {
			r.log("Transmit(): done!")
			break
		}
		r.log("Transmit(): not yet...")
		time.Sleep(5 * time.Millisecond)
	}
	return nil
}

// logRegs is a debug helper function to print almost all the sx1231's registers.
func (r *Radio) logRegs() {
	var buf, regs [0x60]byte
	buf[0] = 1
	r.spi.Tx(buf[:], regs[:])
	regs[0] = 0 // no real data there
	r.log("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F")
	for i := 0; i < len(regs); i += 16 {
		line := fmt.Sprintf("%02x:", i)
		for j := 0; j < 16 && i+j < len(regs); j++ {
			line += fmt.Sprintf(" %02x", regs[i+j])
		}
		r.log(line)
	}
}

// writeReg writes one or multiple registers starting at addr, the sx1231 auto-increments (except
// for the FIFO register where that wouldn't be desirable).
func (r *Radio) writeReg(addr byte, data ...byte) {
	wBuf := make([]byte, len(data)+1)
	rBuf := make([]byte, len(data)+1)
	wBuf[0] = addr | 0x80
	copy(wBuf[1:], data)
	r.spi.Tx(wBuf[:], rBuf[:])
}

// readReg reads one register and returns its value.
func (r *Radio) readReg(addr byte) byte {
	var buf [2]byte
	r.spi.Tx([]byte{addr & 0x7f, 0}, buf[:])
	return buf[1]
}

// readReg16 reads one 16-bit register and returns its value.
func (r *Radio) readReg16(addr byte) uint16 {
	var buf [3]byte
	r.spi.Tx([]byte{addr & 0x7f, 0, 0}, buf[:])
	return (uint16(buf[1]) << 8) | uint16(buf[2])
}
