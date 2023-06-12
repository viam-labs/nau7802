// Package nau7802module implements a nau7802 sensor for load cell amplification
// datasheet can be found at: https://cdn.sparkfun.com/assets/e/c/8/7/6/NAU7802_Data_Sheet_V1.7.pdf
// example repo: https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library
package nau7802module

import (
	"context"
	"encoding/hex"
	"errors"
	"fmt"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/utils"
	"go.uber.org/multierr"

	"go.viam.com/rdk/components/board"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/resource"
)

var Model = resource.DefaultModelFamily.WithModel("nau7802")

// Non-iota consts
const (
	defaultI2Caddr = 0x2A
	defaultSamples = 4
	defaultGain = 64
	
	NAU7802DEVICEREV = 0x1F
)

// AttrConfig is used for converting config attributes.
type Config struct {
	I2CBus  string `json:"i2c_bus"`
	I2cAddr int    `json:"i2c_addr,omitempty"`
	Gain    int    `json:"gain,omitempty"`
	Samples int    `json:"samples,omitempty"`
}

// Validate ensures all parts of the config are valid.
func (config *Config) Validate(path string) ([]string, error) {
	var deps []string
	if len(config.I2CBus) == 0 {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "i2c_bus")
	}
	return deps, nil
}

func init() {
	resource.RegisterComponent(
		sensor.API,
		Model,
		resource.Registration[sensor.Sensor, *Config]{
			Constructor: func(
				ctx context.Context,
				deps resource.Dependencies,
				conf resource.Config,
				logger golog.Logger,
			) (sensor.Sensor, error) {
				newConf, err := resource.NativeConfig[*Config](conf)
				if err != nil {
					return nil, err
				}
				return newSensor(ctx, deps, conf.ResourceName(), newConf, logger)
		},
	})
}

func newSensor(
	ctx context.Context,
	deps resource.Dependencies,
	name resource.Name,
	attr *Config,
	logger golog.Logger,
) (sensor.Sensor, error) {
	var err error
	
	i2cbus, err := newI2cBus(attr.I2CBus)
	if err != nil {
		return nil, err
	}
	addr := attr.I2cAddr
	if addr == 0 {
		addr = defaultI2Caddr
		logger.Warnf("using i2c address : 0x%s", hex.EncodeToString([]byte{byte(addr)}))
	}
	
	samples := attr.Samples
	if samples == 0 {
		samples = defaultSamples
	}

	s := &nau7802{
		Named:    name.AsNamed(),
		logger: logger,
		bus:    i2cbus,
		addr:   byte(addr),
		samples: samples,
	}
	
	s.gain = attr.Gain
	if s.gain == 0 {
		s.gain = defaultGain
	}
	// Full reset and re-init
	err = s.resetCycle(ctx)
	if err != nil {
		return nil, err
	}
	
	time.Sleep(50 * time.Millisecond)
	err = s.calculateZeroOffset(ctx, s.samples)
	if err != nil {
		return nil, err
	}


	return s, nil
}

// nau7802 is a i2c sensor device that reports temperature and humidity.
type nau7802 struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable
	logger golog.Logger

	bus  board.I2C
	addr byte
	samples int
	
	zeroOffset int
	calibrationFactor float64
	gain int
}

// Readings returns a list containing two items (current temperature and humidity).
func (s *nau7802) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	mass, raw, err := s.getWeight(ctx, false, s.samples)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"raw_value": raw,
		"mass_kg": mass,
	}, nil
}

func (s *nau7802) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if val, ok := cmd["calibrate"]; ok {
		if floatVal, ok := val.(float64); ok {
			//Re-cal analog front end when we change gain, sample rate, or channel
			err := s.calculateCalibrationFactor(ctx, floatVal, s.samples)
			if err != nil {
				return nil, err
			}
		}
	}
	if _, ok := cmd["zero"]; ok {
		err := s.calculateZeroOffset(ctx, s.samples)
		if err != nil {
			return nil, err
		}
	}
	if _, ok := cmd["reset"]; ok {
		err := s.resetCycle(ctx)
		if err != nil {
			return nil, err
		}
	}
	return nil, nil
}

//Returns true if Cycle Ready bit is set (conversion is complete)
func (s *nau7802) resetCycle(ctx context.Context) error {
	err := s.reset(ctx); //Reset all registers
	if err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)
	err = s.powerUp(ctx); //Power on analog and digital sections of the scale
	if err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)
	err = s.setLDO(ctx, NAU7802_LDO_3V0); //Set LDO to 3.0V, 0.3V less than input per datasheet
	if err != nil {
		return err
	}
	var gainByte byte
	switch s.gain {
	case 1:
		gainByte = NAU7802_GAIN_1
	case 2:
		gainByte = NAU7802_GAIN_2
	case 4:
		gainByte = NAU7802_GAIN_4
	case 8:
		gainByte = NAU7802_GAIN_8
	case 16:
		gainByte = NAU7802_GAIN_16
	case 32:
		gainByte = NAU7802_GAIN_32
	case 64:
		gainByte = NAU7802_GAIN_64
	case 128:
		gainByte = NAU7802_GAIN_128
	default:
		return fmt.Errorf("%v is not a valid gain value. Choose from 1,2,4,8,16,32,64,128", s.gain)
	}
	
	err = s.setGain(ctx, gainByte); //Set gain
	if err != nil {
		return err
	}
	err = s.setSampleRate(ctx, NAU7802_SPS_10); //Set samples per second to 10
	if err != nil {
		return err
	}
	err = s.setRegister(ctx, NAU7802ADCREG, 0x30); //Turn off CLK_CHP. From 9.1 power on sequencing.
	if err != nil {
		return err
	}
	err = s.setBit(ctx, 0x07, NAU7802PGAPWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
	if err != nil {
		return err
	}
	time.Sleep(50 * time.Millisecond)
	err = s.calibrateAFE(ctx); //Re-cal analog front end when we change gain, sample rate, or channel
	if err != nil {
		return err
	}
	return nil
}

//Returns true if Cycle Ready bit is set (conversion is complete)
func (s *nau7802) available(ctx context.Context) (bool, error) {
  return s.getBit(ctx, NAU7802_CR, NAU7802PUCTRL)
}

//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
func (s *nau7802) calibrateAFE(ctx context.Context) error {
  s.beginCalibrateAFE(ctx);
  return s.waitForCalibrateAFE(ctx, 1000);
}

//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
func (s *nau7802) beginCalibrateAFE(ctx context.Context) error {
  return s.setBit(ctx, NAU7802_CALS, NAU7802CTRL2);
}

//Check calibration status.
func (s *nau7802) calAFEStatus(ctx context.Context) (calibrationStatus, error) {
	calBit, err := s.getBit(ctx, NAU7802_CALS, NAU7802CTRL2)
	if err != nil {
		return NAU7802_CAL_FAILURE, err
	}
	if calBit {
		return NAU7802_CAL_IN_PROGRESS, nil
	}

	calErrBit, err := s.getBit(ctx, NAU7802_CAL_ERR, NAU7802CTRL2)
	if calErrBit || err != nil {
		return NAU7802_CAL_FAILURE, err
	}

	// Calibration passed
	return NAU7802_CAL_SUCCESS, nil
}

//Wait for asynchronous AFE calibration to complete with optional timeout.
//If timeout is not specified (or set to 0), then wait indefinitely.
//Returns true if calibration completes succsfully, otherwise returns false.
func (s *nau7802) waitForCalibrateAFE(ctx context.Context, timeout_ms int) error {
	start := time.Now()
	var err error
	calReady := NAU7802_CAL_IN_PROGRESS

	for calReady == NAU7802_CAL_IN_PROGRESS{
		calReady, err = s.calAFEStatus(ctx)
		if err != nil {
			return err
		}
		if timeout_ms > 0 && time.Since(start) > time.Duration(timeout_ms) * time.Millisecond {
			break
		}
		time.Sleep(1 * time.Millisecond)
	}

	if (calReady == NAU7802_CAL_SUCCESS){
		return nil
	}
	return errors.New("failed to calibrate nau7802")
}

//Set the readings per second
//10, 20, 40, 80, and 320 samples per second is available
func (s *nau7802) setSampleRate(ctx context.Context, rate byte) error {
	if (rate > 0b111) {
		rate = 0b111; //Error check
	}

	value, err := s.getRegister(ctx, NAU7802CTRL2)
	if err != nil {
		return err
	}
	value &= 0b10001111; //Clear CRS bits
	value |= rate << 4;  //Mask in new CRS bits

	return s.setRegister(ctx, NAU7802CTRL2, value)
}

//Select between channels 1 and 2
func (s *nau7802) setChannel(ctx context.Context, channelNumber byte) error {
	if (channelNumber == 0) {
		return s.clearBit(ctx, NAU7802_CHS, NAU7802CTRL2) //Channel 1 (default)
	}else{
		return s.setBit(ctx, NAU7802_CHS, NAU7802CTRL2) //Channel 2
	}
}

//Power up digital and analog sections of scale
func (s *nau7802) powerUp(ctx context.Context) error {
	err := s.setBit(ctx, NAU7802_PUD, NAU7802PUCTRL);
	if err != nil {
		return err
	}
	s.setBit(ctx, NAU7802_PUA, NAU7802PUCTRL);

	//Wait for Power Up bit to be set - takes approximately 200us
	for i := 0; i < 100; i++{
		pwrBit, err := s.getBit(ctx, NAU7802_PUR, NAU7802PUCTRL)
		if err != nil {
			return err
		}
		if pwrBit == true {
			return nil
		}
		time.Sleep(1 * time.Millisecond)
	}
	return errors.New("failed to power up nau7802")
}

//Puts scale into low-power mode
func (s *nau7802) powerDown(ctx context.Context) error {
	return multierr.Combine(s.clearBit(ctx, NAU7802_PUA, NAU7802PUCTRL), s.clearBit(ctx, NAU7802_PUD, NAU7802PUCTRL))
}

//Resets all registers to Power Of Defaults
func (s *nau7802) reset(ctx context.Context) error {
	err := s.setBit(ctx, NAU7802_RR, NAU7802PUCTRL); //Set RR
	if err != nil {
		return err
	}
	time.Sleep(1 * time.Millisecond)
	return s.clearBit(ctx, NAU7802_RR, NAU7802PUCTRL) //Clear RR to leave reset state
}

//Set the onboard Low-Drop-Out voltage regulator to a given value
//2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
func (s *nau7802) setLDO(ctx context.Context, ldoValue byte) error {
	if (ldoValue > 0b111) {
		ldoValue = 0b111; //Error check
	}

	//Set the value of the LDO
	value, err := s.getRegister(ctx, NAU7802CTRL1)
	if err != nil {
		return err
	}
	value &= 0b11000111;    //Clear LDO bits
	value |= ldoValue << 3; //Mask in new LDO bits
	err = s.setRegister(ctx, NAU7802CTRL1, value);
	if err != nil {
		return err
	}

	return s.setBit(ctx, NAU7802_AVDDS, NAU7802PUCTRL) //Enable the internal LDO
}

//Set the gain
//x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
func (s *nau7802) setGain(ctx context.Context, gainValue byte) error {
	if (gainValue > 0b111){
		gainValue = 0b111; //Error check
	}

	value, err := s.getRegister(ctx, NAU7802CTRL1)
	if err != nil {
		return err
	}
	value &= 0b11111000; //Clear gain bits
	value |= gainValue;  //Mask in new bits

	return s.setRegister(ctx, NAU7802CTRL1, value)
}

//Get the revision code of this IC
func (s *nau7802) getRevisionCode(ctx context.Context) (byte, error) {
	revisionCode, err := s.getRegister(ctx, NAU7802DEVICEREV)
	if err != nil {
		return 0, err
	}
	return (revisionCode & 0x0F), nil
}

//Returns 24-bit reading
//Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
func (s *nau7802) getReading(ctx context.Context) (int, error){
	handle, err := s.bus.OpenHandle(s.addr)
	if err != nil {
		return 0, err
	}
	defer handle.Close()
	err = handle.Write(ctx, []byte{NAU7802ADCOB2})
	if err != nil {
		return 0, err
	}

	buffer, err := handle.Read(ctx, 3)
	if err != nil {
		return 0, err
	}
	valueRaw := int32(buffer[0]) << 16
	valueRaw |= int32(buffer[1]) << 8
	valueRaw |= int32(buffer[2])
	// the raw value coming from the ADC is a 24-bit number, so the sign bit now
	// resides on bit 23 (0 is LSB) of the uint32_t container. By shifting the
	// value to the left, the sign bit is moved to the MSB of the uint32_t container.
	// By casting to a signed int32_t container properly recovers
	// the sign of the original value
	valueShifted := uint32(valueRaw << 8)
	
	// shift the number back right to recover its intended magnitude
	return int(valueShifted >> 8), nil
}

//Return the average of a given number of readings
func (s *nau7802) getAverage(ctx context.Context, samplesToTake int) (int, error) {
	total := 0
	samples := 0

	start := time.Now()
	for i := 0; i < samplesToTake; i++{
		availBit, err := s.available(ctx)
		if err != nil {
			return 0, err
		}
		if availBit{
			reading, err := s.getReading(ctx)
			if err != nil {
				return 0, err
			}
			total += reading
			samples++
		}
		if time.Since(start) > time.Duration(150 * samplesToTake) * time.Millisecond{
			return 0, errors.New("timed out reading data from nau7802")
		}
		// Sleep in between each reading to ensure the chip is ready to read again
		time.Sleep(90 * time.Millisecond)
	}
	
	if samples == 0 {
		return 0, nil
	}

	return total/samples, nil
}

//Call when scale is setup, level, at running temperature, with nothing on it
func (s *nau7802) calculateZeroOffset(ctx context.Context, samplesToTake int) error {
	avg, err := s.getAverage(ctx, samplesToTake)
	if err != nil {
		return err
	}
	s.setZeroOffset(avg)
	return nil
}

//Sets the internal variable. Useful for users who are loading values from NVM.
func (s *nau7802) setZeroOffset(newZeroOffset int) {
  s.zeroOffset = newZeroOffset;
}

func (s *nau7802) getZeroOffset() int {
  return s.zeroOffset
}

//Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
func (s *nau7802) calculateCalibrationFactor(ctx context.Context, weightOnScale float64, samplesToTake int) error {
	onScale, err := s.getAverage(ctx, samplesToTake);
	if err != nil {
		return err
	}
	newCalFactor := float64(onScale - s.zeroOffset) / weightOnScale;
	s.setCalibrationFactor(newCalFactor)
	return nil
}

//Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
//If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
func (s *nau7802) setCalibrationFactor(newCalFactor float64) {
	s.calibrationFactor = newCalFactor;
}

func (s *nau7802) getCalibrationFactor() float64 {
	return s.calibrationFactor
}

//Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
func (s *nau7802) getWeight(ctx context.Context, allowNegativeWeights bool, samplesToTake int) (float64, int, error){
	onScale, err := s.getAverage(ctx, samplesToTake)
	if err != nil {
		return 0, 0, err
	}

	//Prevent the current reading from being less than zero offset
	//This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
	//causing the weight to be negative or jump to millions of pounds
	if (allowNegativeWeights == false){
		if (onScale < s.zeroOffset){
			onScale = s.zeroOffset; //Force reading to zero
		}
	}
	weight := float64(onScale - s.zeroOffset) / s.calibrationFactor
	return weight, onScale, nil
}

//Set Int pin to be high when data is ready (default)
func (s *nau7802) setIntPolarityHigh(ctx context.Context) error {
  return s.clearBit(ctx, NAU7802_CRP, NAU7802CTRL1) //0 = CRDY pin is high active (ready when 1)
}

//Set Int pin to be low when data is ready
func (s *nau7802) setIntPolarityLow(ctx context.Context) error {
  return s.setBit(ctx, NAU7802_CRP, NAU7802CTRL1) //1 = CRDY pin is low active (ready when 0)
}

//Mask & set a given bit within a register
func (s *nau7802) setBit(ctx context.Context, bitNumber, registerAddress byte) error {
	value, err := s.getRegister(ctx, registerAddress)
	if err != nil {
		return err
	}
	value |= (1 << bitNumber); //Set this bit
	return s.setRegister(ctx, registerAddress, value)
}

//Mask & clear a given bit within a register
func (s *nau7802) clearBit(ctx context.Context, bitNumber, registerAddress byte) error {
	value, err := s.getRegister(ctx, registerAddress)
	if err != nil {
		return err
	}
	value &= ^(1 << bitNumber); //Set this bit
	return s.setRegister(ctx, registerAddress, value)
}

//Return a given bit within a register
func (s *nau7802) getBit(ctx context.Context, bitNumber, registerAddress byte) (bool, error){
	value, err := s.getRegister(ctx, registerAddress);
	if err != nil {
		return false, err
	}
	value &= (1 << bitNumber); //Clear all but this bit
	return value > 0, nil
}

//Get contents of a register
func (s *nau7802) getRegister(ctx context.Context, registerAddress byte) (byte, error) {
	handle, err := s.bus.OpenHandle(s.addr)
	if err != nil {
		return 0, err
	}
	err = handle.Write(ctx, []byte{registerAddress})
	if err != nil {
		return 0, err
	}
	buffer, err := handle.Read(ctx, 1)
	if err != nil {
		return 0, err
	}
	return buffer[0], handle.Close()
}

//Send a given value to be written to given address
//Return true if successful
func (s *nau7802) setRegister(ctx context.Context, registerAddress, value byte) error {
	handle, err := s.bus.OpenHandle(s.addr)
	if err != nil {
		return err
	}
	err = handle.Write(ctx, []byte{registerAddress, value})
	if err != nil {
		return err
	}
	return handle.Close()
}
