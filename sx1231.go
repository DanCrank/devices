// Copyright 2016 by Thorsten von Eicken, see LICENSE file

// The SX1231 package interfaces with a HopeRF RFM69 radio connected to an SPI bus.
//
// The RFM69 modules use a Semtech SX1231 or SX1231H radio chip and this
// package should work fine with other radio modules using the same chip. The only real
// difference will be the power output section where different modules use different output stage
// configurations.
//
// The driver is fully interrupt driven and requires that the radio's DIO0 pin be connected to
// an interrupt capable GPIO pin. The transmit and receive interface uses a pair of tx and rx
// channels, each having a small amount of buffering.
//
// In general, other than a few user errors (such as passing too large a packet to Send) there
// should be no errors during the radio's operation unless there is a hardware failure. For this
// reason radio interface errors are treated as fatal: if such an error occurs the rx channel is
// closed and the error is recorded in the Radio struct where it can be retrieved using the Error
// function. The object will be unusable for further operation and the client code will have to
// create and initialize a fresh object which will re-establish communication with the radio chip.
//
// This driver does not do a number of things that other sx1231 drivers tend to do with the
// goal of leaving these tasks to higher-level drivers. This driver does not use the address
// filtering capability: it recevies all packets because that's simpler and the few extra interrupts
// should not matter to a system that can run Golang. It also accepts packets that have a CRC error
// and simply flags the error. It does not constrain the sync bytes, the frequency, or the data
// rates.
//
// The main limitations of this driver are that it operates the sx1231 in FSK variable-length packet
// mode and limits the packet size to the 66 bytes that fit into the FIFO, meaning that the payloads
// pushed into the TX channel must be 65 bytes or less, leaving one byte for the required packet
// length.
//
// The methods on the Radio object are not concurrency safe. Since they all deal with configuration
// this should not pose difficulties. The Error function may be called from multiple goroutines
// and obviously the TX and RX channels work well with concurrency.
//
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
	intrCnt int        // count interrupts
	sync    []byte     // sync bytes
	freq    uint32     // center frequency
	rate    uint32     // bit rate from table
	paBoost bool       // true: use PA1+PA2 power amp, else PA0
	power   byte       // output power in dBm
	// state
	sync.Mutex           // guard concurrent access to the radio
	mode       byte      // current operation mode
	rxTimeout  uint32    // RX timeout counter to tune rssi threshold
	rssiAdj    time.Time // when the rssi threshold was last adjusted
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
	49230: {45000, 0, 0x4A, 0x42},  // JeeLabs driver for rfm69 (RxBw=100, AfcBw=125)
	49231: {180000, 0, 0x49, 0x49}, // JeeLabs driver with rf12b compatibility
	49232: {45000, 0, 0x52, 0x4A},  // JeeLabs driver for rfm69 (RxBw=83, AfcBw=100)
	49233: {51660, 0, 0x52, 0x4A},  // JeeLabs driver for rfm69 (RxBw=83, AfcBw=100)
	50000: {90000, 0, 0x42, 0x42},  // nice round number
}

// TODO: check whether the following is a better setting for 50kbps:
// freescale app note http://cache.nxp.com/files/rf_if/doc/app_note/AN4983.pdf?fpsp=1&WT_TYPE=Application%20Notes&WT_VENDOR=FREESCALE&WT_FILE_FORMAT=pdf&WT_ASSET=Documentation&fileExt=.pdf
// uses: 50kbps, 25khz Fdev, 1.0 gaussian filter, 156.24khz RxBW&AfcBW, 3416Hz LO offset, 497Hz DCC

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

type busyError struct{ e string }

func (b busyError) Error() string   { return b.e }
func (b busyError) Temporary() bool { return true }

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
//
// To transmit, push packet payloads into the returned txChan.
// Received packets will be sent on the returned rxChan, which has a small amount of
// buffering. The rxChan will be closed if a persistent error occurs when
// communicating with the device, use the Error() function to retrieve the error.
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
	r.SetPower(13)

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

	count := 0
repeat:
	// Initialize interrupt pin.
	if err := r.intrPin.In(gpio.Float, gpio.RisingEdge); err != nil {
		return nil, fmt.Errorf("sx1231: error initializing interrupt pin: %s", err)
	}
	r.log("Interrupt pin is %v", r.intrPin.Read())

	// Test the interrupt function by configuring the radio such that it generates an interrupt
	// and then call WaitForEdge. Start by verifying that we don't have any pending interrupt.
	for r.intrPin.WaitForEdge(0) {
		r.log("Interrupt test shows an incorrect pending interrupt")
	}
	// Make the radio produce an interrupt.
	r.setMode(MODE_FS)
	r.writeReg(REG_DIOMAPPING1, DIO_MAPPING+0xC0)
	if !r.intrPin.WaitForEdge(100 * time.Millisecond) {
		if count == 0 {
			r.writeReg(REG_DIOMAPPING1, DIO_MAPPING)
			//r.intrPin.Close()
			time.Sleep(100 * time.Millisecond)
			count++
			goto repeat
		}
		return nil, fmt.Errorf("sx1231: interrupts from radio do not work, try unexporting gpio%d", r.intrPin.Number())
	}
	r.writeReg(REG_DIOMAPPING1, DIO_MAPPING)
	// Flush any addt'l interrupts.
	for r.intrPin.WaitForEdge(0) {
	}

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

//

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

// busy checks whether a transmission or a reception is currently in progress.
// For rx it uses the sync match flag as earliest indication that something is coming
// in that is not noise. It also protects from a packet sitting in RX that hasn't been
// picked-up yet by the interrupt handler.
func (r *Radio) busy() bool {
	switch {
	case r.mode == MODE_TRANSMIT:
		return true
	case r.mode == MODE_RECEIVE:
		irq1 := r.readReg(REG_IRQFLAGS1)
		return irq1&IRQ1_SYNCMATCH != 0
	default:
		return false
	}
}

// trying a simplified receive function based on what the Rust driver does.
// if this works I will replace Receive() with this.
func (r *Radio) SimpleReceive(timeout time.Duration) (*RxPacket, error) {
	// set receive mode
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
		return nil, nil
	}
	// set standby mode
	r.setMode(MODE_STANDBY)
	// read packet from fifo
	var rssi, fei int
	// Bail out if we're not actually receiving a packet. This happens when the
	// receiver restarts because RSSI went away or no SYNC was found before timeout.
	irq1 := r.readReg(REG_IRQFLAGS1)
	if irq1&(IRQ1_RXREADY|IRQ1_RSSI) != IRQ1_RXREADY|IRQ1_RSSI {
		//r.log("... not receiving? IRQ=%t mode=%#02x irq1=%#02x irq2=%02x",
		//	r.intrPin.Read(), r.readReg(REG_OPMODE), irq1, irq2)
		return nil, nil
	}
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
	return &RxPacket{Payload: buf[1 : 1+l], Rssi: rssi, Snr: snr, Fei: fei}, nil
}

func (r *Radio) Receive() (*RxPacket, error) {
	// TODO: timeout
	// If we haven't yet, start the timer for RSSI threshold adjustments.
	if r.rssiAdj.IsZero() {
		r.rxTimeout = 0
		r.rssiAdj = time.Now()
	}

	r.Lock()
	defer r.Unlock()

	// Loop over interrupts & timeouts.
	for {
		// Make sure we're not missing an initial edge due to a race condition.
		intr := r.intrPin.Read() == gpio.High

		if !intr {
			r.Unlock()
			intr = r.intrPin.WaitForEdge(1 * time.Second)
			r.Lock()
		}

		if !intr && r.intrPin.Read() == gpio.High {
			// Sometimes WaitForEdge times out yet the interrupt pin is
			// active, this means the driver or epoll failed us.
			// Need to understand this better.
			r.log("Interrupt was missed!")
			// If we don't get interrupts it messes with the rx threshold adjustemnt.
			r.rxTimeout = 0
			r.rssiAdj = time.Now()
		}
		intr = false

		if r.intrPin.Read() == gpio.High {
			switch {
			case r.mode == MODE_RECEIVE:
				pkt, err := r.rx()
				r.setMode(MODE_RECEIVE)
				if pkt != nil || err != nil {
					return pkt, err
				}
			case r.mode == MODE_TRANSMIT:
				r.txDone()
			default:
				r.setMode(MODE_RECEIVE) // clears intr
			}
		}

		// If we're in RX mode and the chip shows a timeout, then reset it.
		// Not sure why it happens, perhaps if WaitForEdge times out and then
		// the RX timeout happens before we get here?
		if r.mode == MODE_RECEIVE {
			if irq1 := r.readReg(REG_IRQFLAGS1); irq1&IRQ1_TIMEOUT != 0 {
				//r.log("Rx restart -- mode: %#x, mapping: %#x, IRQ flags: %#x %#x",
				//	r.readReg(REG_OPMODE), r.readReg(REG_DIOMAPPING1),
				//	irq1, r.readReg(REG_IRQFLAGS2))
				r.setMode(MODE_FS)
				r.setMode(MODE_RECEIVE)
			}
		}
		// Adjust RSSI threshold
		if dt := time.Since(r.rssiAdj); dt > 10*time.Second {
			timeoutPerSec := float64(r.rxTimeout) / dt.Seconds()
			switch {
			case timeoutPerSec > 10:
				r.writeReg(REG_RSSITHRES, r.readReg(REG_RSSITHRES)-1)
				r.log("RSSI threshold raised: %.2f timeout/sec, %.1fdBm",
					timeoutPerSec, -float64(r.readReg(REG_RSSITHRES))/2)
			case timeoutPerSec < 5:
				r.writeReg(REG_RSSITHRES, r.readReg(REG_RSSITHRES)+1)
				thres := -float64(r.readReg(REG_RSSITHRES)) / 2
				if thres < -105 {
					// This is getting absurd, something is not working here
					// let's reset to a more reasonable value.
					r.writeReg(REG_RSSITHRES, 2*95)
					r.log("RSSI threshold reset: %.2f timeout/sec, %.1fdBm",
						timeoutPerSec, -float64(r.readReg(REG_RSSITHRES))/2)
				} else {
					r.log("RSSI threshold lowered: %.2f timeout/sec, %.1fdBm",
						timeoutPerSec, -float64(r.readReg(REG_RSSITHRES))/2)
				}
			}
			r.rxTimeout = 0
			r.rssiAdj = time.Now()
		}
	}
}

// Transmit switches the radio's mode and starts transmitting a packet.
func (r *Radio) Transmit(payload []byte) error {
	r.Lock()
	defer r.Unlock()

	if r.busy() {
		return busyError{"radio is busy"}
	}
	// limit the payload to valid lengths
	switch {
	case len(payload) > 65:
		payload = payload[:65]
	case len(payload) == 0:
		return errors.New("invalid payload length")
	}
	r.setMode(MODE_FS)
	//r.writeReg(0x2D, 0x01) // set preamble to 1 (too short)
	//r.writeReg(0x2F, 0x00) // set wrong sync value

	// push the message into the FIFO.
	buf := make([]byte, len(payload)+1)
	buf[0] = byte(len(payload))
	copy(buf[1:], payload)
	r.writeReg(REG_FIFO|0x80, buf...)
	r.setMode(MODE_TRANSMIT)
	return nil
}

// txDone handles an interrupt after transmitting.
func (r *Radio) txDone() {
	// Double-check that the packet got transmitted.
	if irq2 := r.readReg(REG_IRQFLAGS2); irq2&IRQ2_PACKETSENT == 0 {
		r.log("TX done interrupt, but packet not transmitted? %#x", irq2)
	}
	//r.log("TX done")
	// Now receive.
	r.setMode(MODE_RECEIVE)
}

// rx handles a receive interrupt. See the notes in the README about the various
// possible strategies.
func (r *Radio) rx() (*RxPacket, error) {
	// Get timestamp and calculate timeout. At 50kbps a 2-byte ACK takes 2ms and a full 66 byte
	// packet takes 12.3ms.
	t0 := time.Now()
	tOut := t0.Add(time.Second * 80 * 8 / time.Duration(r.rate)) // time for 80 bytes

	// Helper function to empty FIFO. It's faster to read the whole thing than to first look at
	// the length.
	readFifo := func() []byte {
		var wBuf, rBuf [67]byte
		wBuf[0] = REG_FIFO
		r.spi.Tx(wBuf[:], rBuf[:])
		return rBuf[1:]
	}

	// Loop until we have the full packet, or things go south. Grab RSSI & AFC after
	// sync match and only if we can get them before the packet is fully received.
	var rssi, fei int
	for {
		// See whether we have a full packet.
		irq2 := r.readReg(REG_IRQFLAGS2)
		if irq2&IRQ2_PAYLOADREADY != 0 {
			if irq2&IRQ2_CRCOK == 0 {
				r.log("Rx bad CRC")
				readFifo()
				return nil, nil
			}
			if rssi == 0 {
				r.log("Rx interrupt: packet was ready")
			}
			break
		}
		// Bail out if we're not actually receiving a packet. This happens when the
		// receiver restarts because RSSI went away or no SYNC was found before timeout.
		irq1 := r.readReg(REG_IRQFLAGS1)
		if irq1&(IRQ1_RXREADY|IRQ1_RSSI) != IRQ1_RXREADY|IRQ1_RSSI {
			//r.log("... not receiving? IRQ=%t mode=%#02x irq1=%#02x irq2=%02x",
			//	r.intrPin.Read(), r.readReg(REG_OPMODE), irq1, irq2)
			return nil, nil
		}
		// As soon as we have sync match, grab RSSI and FEI.
		if rssi == 0 && irq1&IRQ1_SYNCMATCH != 0 {
			// Get RSSI.
			rssi = 0 - int(r.readReg(REG_RSSIVALUE))/2
			// Get freq error detected, caution: signed 16-bit value.
			f := int(int16(r.readReg16(REG_AFCMSB)))
			fei = (f * (32000000 >> 13)) >> 6
		}
		// Timeout so we don't get stuck here.
		if time.Now().After(tOut) {
			//r.log("RX timeout! irq1=%#02x irq2=%02x, rssi=%ddBm fei=%dHz", irq1, irq2,
			//	0-int(r.readReg(REG_RSSIVALUE))/2,
			//	(int(int16(r.readReg16(REG_AFCMSB)))*(32000000>>13))>>6)
			r.rxTimeout++
			// Make sure the FIFO is empty (not sure this is necessary).
			if irq2&IRQ2_FIFONOTEMPTY != 0 {
				//r.log("RX timeout! irq1=%#02x irq2=%02x, rssi=%ddBm afc=%dHz", irq1, irq2,
				//	0-int(r.readReg(REG_RSSIVALUE))/2,
				//	(int(int16(r.readReg16(REG_AFCMSB)))*(32000000>>13))>>6)
				readFifo()
				//r.log("FIFO: %+v", readFifo())
			}
			// Restart Rx.
			r.writeReg(REG_PKTCONFIG2, 0x16)
			return nil, nil
		}
		time.Sleep(time.Millisecond)
	}

	// Got packet, read it.
	buf := readFifo()

	// Construct RxPacket and return it.
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
	return &RxPacket{Payload: buf[1 : 1+l], Rssi: rssi, Snr: snr, Fei: fei}, nil
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
