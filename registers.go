// Copyright 2016 by Thorsten von Eicken, see LICENSE file

package sx1231

const (
	REG_FIFO        = 0x00
	REG_OPMODE      = 0x01
	REG_DATAMODUL   = 0x02
	REG_BITRATEMSB  = 0x03
	REG_FDEVMSB     = 0x05
	REG_FRFMSB      = 0x07
	REG_AFCCTRL     = 0x0B
	REG_VERSION     = 0x10
	REG_PALEVEL     = 0x11
	REG_LNAVALUE    = 0x18
	REG_RXBW        = 0x19
	REG_AFCBW       = 0x1A
	REG_AFCFEI      = 0x1E
	REG_AFCMSB      = 0x1F
	REG_AFCLSB      = 0x20
	REG_FEIMSB      = 0x21
	REG_FEILSB      = 0x22
	REG_RSSICONFIG  = 0x23
	REG_RSSIVALUE   = 0x24
	REG_DIOMAPPING1 = 0x25
	REG_IRQFLAGS1   = 0x27
	REG_IRQFLAGS2   = 0x28
	REG_RSSITHRES   = 0x29
	REG_SYNCCONFIG  = 0x2E
	REG_SYNCVALUE1  = 0x2F
	REG_SYNCVALUE2  = 0x30
	REG_NODEADDR    = 0x39
	REG_BCASTADDR   = 0x3A
	REG_FIFOTHRESH  = 0x3C
	REG_PKTCONFIG2  = 0x3D
	REG_AESKEYMSB   = 0x3E
	REG_TESTPA1     = 0x5A
	REG_TESTPA2     = 0x5C
	REG_TESTAFC     = 0x71

	MODE_SLEEP    = 0 << 2
	MODE_STANDBY  = 1 << 2
	MODE_FS       = 2 << 2
	MODE_TRANSMIT = 3 << 2
	MODE_RECEIVE  = 4 << 2

	START_TX = 0xC2
	STOP_TX  = 0x42

	RCCALSTART     = 0x80
	IRQ1_MODEREADY = 1 << 7
	IRQ1_RXREADY   = 1 << 6
	IRQ1_PLLLOCK   = 1 << 4
	IRQ1_RSSI      = 1 << 3
	IRQ1_TIMEOUT   = 1 << 2
	IRQ1_SYNCMATCH = 1 << 0

	IRQ2_FIFONOTEMPTY = 1 << 6
	IRQ2_PACKETSENT   = 1 << 3
	IRQ2_PAYLOADREADY = 1 << 2
	IRQ2_CRCOK        = 1 << 1

	DIO_MAPPING  = 0x31
	DIO_RSSI     = 0xC0
	DIO_SYNC     = 0x80
	DIO_PAYREADY = 0x40
	DIO_PKTSENT  = 0x00
)

// register values to initialize the chip, this array has pairs of <address, data>
var configRegs = []byte{
	0x01, 0x00, // OpMode = sleep
	0x05, 0x01, // FdevMsb
	0x06, 0x3B, // FdevLsb
	0x11, 0x9F, // power output
	0x12, 0x09, // Pa ramp in 40us
	0x1E, 0x0C, // AfcAutoclearOn, AfcAutoOn
	0x25, DIO_MAPPING, // DioMapping1
	0x26, 0x07, // disable clkout
	0x29, 0xA8, // RssiThresh (A0=-80dB, B4=-90dB, B8=-92dB)
	0x2A, 0x00, // disable RxStart timeout
	0x2B, 0x40, // RssiTimeout after 2*64=128 bytes
	0x2D, 0x04, // PreambleSize = 4
	0x37, 0xD0, // PacketConfig1 = variable, white, no filtering, crc autoclear, no addr filter
	0x38, 0x40, // PayloadLength = max 64
	0x3C, 0x8F, // FifoTresh, not empty, level 15
	0x3D, 0x12, // PacketConfig2, interpkt = 1, autorxrestart on
	0x6F, 0x30, // RegTestDagc 20->improve AFC w/low-beta, 30->w/out low-beta offset

	// The settings below are now done dynamically in SetRate, SetFrequency and the sync bytes.
	//0x02, 0x00, // DataModul = packet mode, fsk
	//0x03, 0x02, // BitRateMsb, data rate = 49,261 khz
	//0x04, 0x8A, // BitRateLsb, divider = 32 MHz / 650
	//0x19, 0x4A, // RxBw 100 KHz
	//0x1A, 0x42, // AfcBw 125 KHz
	//0x2E, 0x88, // SyncConfig = sync on, sync size = 2
	//0x2F, 0x2D, // SyncValue1 = 0x2D
	//0x71, 0x02, // RegTestAfc: low-beta opt
}
