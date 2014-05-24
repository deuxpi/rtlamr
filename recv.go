// RTLAMR - An rtl-sdr receiver for smart meters operating in the 900MHz ISM band.
// Copyright (C) 2014 Douglas Hall
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

package main

import (
	"errors"
	"fmt"
	"log"
	"math"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"time"

	"github.com/bemasher/rtltcp"

	"github.com/bemasher/rtlamr/bch"
	"github.com/bemasher/rtlamr/preamble"
)

const (
	BlockSize = 1 << 12

	DataRate     = 32768
	SymbolLength = 73
	SampleRate   = DataRate * SymbolLength

	PreambleSymbols = 64
	PreambleLength  = PreambleSymbols * SymbolLength

	PacketSymbols = 1472
	PacketLength  = PacketSymbols * SymbolLength

	PreambleDFTSize = 8192

	CenterFreq    = 920299072
	RestrictLocal = false

	Preamble     = 0x555516A3
	PreambleBits = "01010101010101010001011010100011"

	GenPoly = 0x16F63

	TimeFormat = "2006-01-02T15:04:05.000"
)

var config Config

type Receiver struct {
	rtltcp.SDR

	pd  preamble.PreambleDetector
	bch bch.BCH
	lut MagLUT
}

func NewReceiver(blockSize int) (rcvr Receiver) {
	// Plan the preamble detector.
	rcvr.pd = preamble.NewPreambleDetector(PreambleDFTSize, SymbolLength, PreambleBits)

	// Create a new BCH for error detection.
	rcvr.bch = bch.NewBCH(GenPoly)
	if !config.Quiet {
		config.Log.Printf("BCH: %+v\n", rcvr.bch)
	}

	rcvr.lut = NewMagLUT()

	// Connect to rtl_tcp server.
	if err := rcvr.Connect(config.ServerAddr); err != nil {
		config.Log.Fatal(err)
	}

	// Tell the user how many gain settings were reported by rtl_tcp.
	if !config.Quiet {
		config.Log.Println("GainCount:", rcvr.SDR.Info.GainCount)
	}

	// Set some parameters for listening.
	rcvr.SetSampleRate(SampleRate)
	rcvr.SetCenterFreq(uint32(config.CenterFreq))
	rcvr.SetGainMode(true)

	return
}

// Clean up rtl_tcp connection and destroy preamble detection plan.
func (rcvr *Receiver) Close() {
	rcvr.SDR.Close()
	rcvr.pd.Close()
}

func (rcvr *Receiver) Run() {
	// Setup signal channel for interruption.
	sigint := make(chan os.Signal, 1)
	signal.Notify(sigint, os.Kill, os.Interrupt)

	// Allocate sample and demodulated signal buffers.
	raw := make([]byte, (PacketLength+BlockSize)<<1)
	amBuf := make([]float64, PacketLength+BlockSize)

	// Setup time limit channel
	tLimit := make(<-chan time.Time, 1)
	if config.TimeLimit != 0 {
		tLimit = time.After(config.TimeLimit)
	}

	start := time.Now()
	for {
		// Exit on interrupt or time limit, otherwise receive.
		select {
		case <-sigint:
			return
		case <-tLimit:
			fmt.Println("Time Limit Reached:", time.Since(start))
			return
		default:
			copy(raw, raw[BlockSize<<1:])
			copy(amBuf, amBuf[BlockSize:])

			// Read new sample block.
			_, err := rcvr.Read(raw[PacketLength<<1:])
			if err != nil {
				config.Log.Fatal("Error reading samples:", err)
			}

			// AM Demodulate
			block := amBuf[PacketLength:]
			for idx := range block {
				block[idx] = math.Sqrt(rcvr.lut[raw[(idx)<<1]] + rcvr.lut[raw[((idx)<<1)+1]])
			}

			// Detect preamble in first half of demod buffer.
			rcvr.pd.Execute(amBuf)
			align := rcvr.pd.ArgMax()

			// Bad framing, catch message on next block.
			if align > BlockSize {
				continue
			}

			// Filter signal and bit slice enough data to catch the preamble.
			filtered := MatchedFilter(amBuf[align:], PreambleSymbols>>1)
			bits := BitSlice(filtered)

			// If the preamble matches.
			if bits == PreambleBits {
				// Filter, slice and parse the rest of the buffered samples.
				filtered := MatchedFilter(amBuf[align:], PacketSymbols>>1)
				bits := BitSlice(filtered)

				// If the checksum fails, bail.
				if rcvr.bch.Encode(bits[16:96]) != 0 {
                                        config.Log.Print("Invalid checksum")
					//continue
				}

				// Parse SCM
				scm, err := ParseSCM(bits)
				if err != nil {
					config.Log.Fatal("Error parsing SCM:", err)
				}

				// If filtering by ID and ID doesn't match, bail.
				//if config.MeterID != 0 && uint(scm.ID) != config.MeterID {
				//	continue
				//}

				// Get current file offset.
				offset, err := config.SampleFile.Seek(0, os.SEEK_CUR)
				if err != nil {
					config.Log.Fatal("Error getting sample file offset:", err)
				}

				// Dump message to file.
				_, err = config.SampleFile.Write(raw)
				if err != nil {
					config.Log.Fatal("Error dumping samples:", err)
				}

				msg := Message{time.Now(), offset, len(raw), scm}

				// Write or encode message to log file.
				if config.Encoder == nil {
					// A nil encoder is just plain-text output.
					fmt.Fprintf(config.LogFile, "%+v", msg)
				} else {
					err = config.Encoder.Encode(msg)
					if err != nil {
						log.Fatal("Error encoding message:", err)
					}

					// The XML encoder doesn't write new lines after each
					// element, add them.
					if strings.ToLower(config.format) == "xml" {
						fmt.Fprintln(config.LogFile)
					}
				}

				if config.Single {
					return
				}
			}
		}
	}
}

// A lookup table for calculating magnitude of an interleaved unsigned byte
// stream.
type MagLUT []float64

// Shifts sample by 127.4 (most common DC offset value of rtl-sdr dongles) and
// stores square.
func NewMagLUT() (lut MagLUT) {
	lut = make([]float64, 0x100)
	for idx := range lut {
		lut[idx] = 127.4 - float64(idx)
		lut[idx] *= lut[idx]
	}
	return
}

// Matched filter implemented as integrate and dump. Output array is equal to
// the number of manchester coded symbols per packet.
func MatchedFilter(input []float64, bits int) (output []float64) {
	output = make([]float64, bits)

	fIdx := 0
	for idx := 0; fIdx < bits; idx += SymbolLength * 2 {
		offset := idx + SymbolLength

		for i := 0; i < SymbolLength; i++ {
			output[fIdx] += input[idx+i] - input[offset+i]
		}
		fIdx++
	}
	return
}

func BitSlice(input []float64) (output string) {
	for _, v := range input {
		if v > 0.0 {
			output += "1"
		} else {
			output += "0"
		}
	}
	return
}

func ParseUint(raw string) uint64 {
	tmp, _ := strconv.ParseUint(raw, 2, 64)
	return tmp
}

// Message for logging.
type Message struct {
	Time   time.Time
	Offset int64
	Length int
	SCM    SCM
}

func (msg Message) String() string {
	// If we aren't dumping samples, omit offset and packet length.
	if config.sampleFilename == os.DevNull {
		return fmt.Sprintf("{Time:%s SCM:%+v}\n",
			msg.Time.Format(TimeFormat), msg.SCM,
		)
	}

	return fmt.Sprintf("{Time:%s Offset:%d Length:%d SCM:%+v}\n",
		msg.Time.Format(TimeFormat), msg.Offset, msg.Length, msg.SCM,
	)
}

// Standard Consumption Message
type SCM struct {
        TrainingSync    uint16
        FrameSync       uint16
        PacketType      uint8
        PacketLength    uint16
        Version         uint8
	Type            uint8
	Serial          uint32
        IntervalCount   uint8
        State           uint8
	Tamper          [6]uint8
        Async           uint16
        Outage          [6]uint8
	LastConsumptionCount uint32
        Intervals       [53]uint8
        Offset          uint16
	SerialChecksum  uint16
        PacketChecksum  uint16
}

func (scm SCM) String() string {
	return fmt.Sprintf("{Sync:0x%04X%04X Type:%2d Length:%2d Version:%2d SCM:{Type:%2d Serial:%8d IntervalCount:%2d Consumption:%8d Checksum:0x%04X} Checksum:0x%04X}",
		scm.TrainingSync, scm.FrameSync, scm.PacketType, scm.PacketLength, scm.Version,
                scm.Type, scm.Serial, scm.IntervalCount, scm.LastConsumptionCount,
                scm.SerialChecksum, scm.PacketChecksum,
	)
}

type Tamper struct {
	Phy uint8
	Enc uint8
        T3 uint8
        T4 uint8
        T5 uint8
        T6 uint8
}

func (t Tamper) String() string {
	return fmt.Sprintf("{Phy:%d Enc:%d}", t.Phy, t.Enc)
}

// Given a string of bits, parse the message.
func ParseSCM(data string) (scm SCM, err error) {
	if len(data) != 736 {
        	return scm, errors.New("invalid input length")
	}

        scm.TrainingSync = uint16(ParseUint(data[:16]))
        scm.FrameSync = uint16(ParseUint(data[16:32]))
        scm.PacketType = uint8(ParseUint(data[32:40]))
        scm.PacketLength = uint16(ParseUint(data[40:56]))
        scm.Version = uint8(ParseUint(data[56:64]))
        scm.Type = uint8(ParseUint(data[64:72]))
        scm.Serial = uint32(ParseUint(data[72:104]))
        scm.IntervalCount = uint8(ParseUint(data[104:112]))
        scm.State = uint8(ParseUint(data[112:120]))
        // Tamper
        scm.Async = uint16(ParseUint(data[168:184]))
        // Outage
        scm.LastConsumptionCount = uint32(ParseUint(data[232:264]))
        // Intervals
        scm.Offset = uint16(ParseUint(data[688:704]))
        scm.SerialChecksum = uint16(ParseUint(data[704:720]))
        scm.PacketChecksum = uint16(ParseUint(data[720:736]))

	return scm, nil
}

func init() {
	err := config.Parse()
	if err != nil {
		config.Log.Fatal("Error parsing flags:", err)
	}
}

func main() {
	if !config.Quiet {
		config.Log.Println("Server:", config.ServerAddr)
		config.Log.Println("BlockSize:", BlockSize)
		config.Log.Println("SampleRate:", SampleRate)
		config.Log.Println("DataRate:", DataRate)
		config.Log.Println("SymbolLength:", SymbolLength)
		config.Log.Println("PreambleSymbols:", PreambleSymbols)
		config.Log.Println("PreambleLength:", PreambleLength)
		config.Log.Println("PacketSymbols:", PacketSymbols)
		config.Log.Println("PacketLength:", PacketLength)
		config.Log.Println("CenterFreq:", config.CenterFreq)
		config.Log.Println("TimeLimit:", config.TimeLimit)

		config.Log.Println("Format:", config.format)
		config.Log.Println("LogFile:", config.logFilename)
		config.Log.Println("SampleFile:", config.sampleFilename)

		if config.MeterID != 0 {
			config.Log.Println("FilterID:", config.MeterID)
		}
	}

	rcvr := NewReceiver(BlockSize)
	defer rcvr.Close()
	defer config.Close()

	if !config.Quiet {
		config.Log.Println("Running...")
	}

	rcvr.Run()
}
