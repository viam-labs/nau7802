package nau7802module
// Register map
const (
	NAU7802PUCTRL byte = iota
	NAU7802CTRL1
	NAU7802CTRL2
	NAU7802OCAL1B2
	NAU7802OCAL1B1
	NAU7802OCAL1B0
	NAU7802GCAL1B3
	NAU7802GCAL1B2
	NAU7802GCAL1B1
	NAU7802GCAL1B0
	NAU7802OCAL2B2
	NAU7802OCAL2B1
	NAU7802OCAL2B0
	NAU7802GCAL2B3
	NAU7802GCAL2B2
	NAU7802GCAL2B1
	NAU7802GCAL2B0
	NAU7802I2CCONTROL
	NAU7802ADCOB2
	NAU7802ADCOB1
	NAU7802ADCOB0
	NAU7802ADCREG //Shared ADC and OTP 32:24
	NAU7802OTPB1     //OTP 23:16 or 7:0?
	NAU7802OTPB0     //OTP 15:8
	NAU7802PGA
	NAU7802PGAPWR
)

//REG0x00:PU_CTRL     Powerup Control
const (
	NAU7802_RR byte = iota  //Register Reset              1:Reset all save RR   0:Normal Operation            (0-Default)
	NAU7802_PUD             //Power up digital circuit    1:Power up digital    0:Power down digital          (0-Default)
	NAU7802_PUA             //Power up analog circuit     1:Power up anlaog     0:Power down analog           (0-Default)
	NAU7802_PUR             //Power up ready (Read Only)  1:Power up, Ready     0:Power down, not ready       (0-Default)
	NAU7802_CS              //Cycle start ADC             1:Start ADC           0:Stop ADC                    (0-Default)
	NAU7802_CR              //Cycle ready (Read Only)     1:ADC Data Ready      0:No ADC Data                 (0-Default)
	NAU7802_OSCS            //System clock source select  1:External Crystal    0:Internal RC Oscillator      (0-Default)
	NAU7802_AVDDS           //AVDD source select          1:Internal LDO        0:AVDD Pin Input              (0-Default)
)

//REG0x11:I2C_CTRL    I2C Control
const (
	NAU7802_BGPCP byte = iota //Bandgap chopper              1:Disable                 0:Enable                  (0-Default)
	NAU7802_TS                //Temprature Sensor Select     1:Temprature to PGA       0:Temp disabled           (0-Default)
	NAU7802_BOPGA             //PGA bunout current source    1:2.5uA Current to PGA+   0:Current disabled        (0-Default)
	NAU7802_SI                //Short Inputs                 1:Short Inputs            0:Inputs floating         (0-Default)
	NAU7802_WPD               //Disable weak pullup          1:Disable 50K pullpup     0:Enable 50K pullup       (0-Default)
	NAU7802_SPE               //Enable strong pullup         1:Enable 1K6 pullup       0:Disable 1K6 pullup      (0-Default)
	NAU7802_FRD               //Fast ADC DATA                1:Enable REQ REG0x15[7]:1 0:Disable                 (0-Default)
	NAU7802_CRSD              //Pull SDA low on conversion   1:Enable                  0:Diable                  (0-Default)
)

//REG0x01:CTRL1       Control 1
const (
	NAU7802_GAINS0 byte = iota  //Select gain                 1      0      1      0     1     0     1     0      (0-Default)
	NAU7802_GAINS1              //Select gain                 1 128  1 x64  0 x32  0 x16 1 x8  1 x4  0 x2  0 x1   (0-Default)
	NAU7802_GAINS2              //Select gain                 1      1      1      1     0     0     0     0      (0-Default)
	NAU7802_VLDO0               //Select LDO Voltage          1      0      1      0     1     0     1     0      (0-Default)
	NAU7802_VLDO1               //Select LDO Voltage          1 2V4  1 2V7  0 3V0  0 3V3 1 3V6 1 3V9 0 4V2 0 4V5  (0-Default)
	NAU7802_VLDO2               //Select LDO Voltage          1      1      1      1     0     0     0     0      (0-Default)
	NAU7802_DRDY_SEL            //DRDY pin fuction            1:Output clock        0:Output conv ready           (0-Default)
	NAU7802_CRP                 //Conversion Ready Polarity   1:Active Low          0:Active high                 (0-Default)
)

//REG0x02:CTRL2       Control 2
const (
	NAU7802_CALMOD0 byte = iota //Calibration Selection        1:System    0:System      1:RESERVED  0: Internal   (0-Default)
	NAU7802_CALMOD1             //Calibration Selection        1:Gain Cal  1:Offset Cal  0:RESERVED  0: Offset Cal (0-Default)
	NAU7802_CALS                //Start Calibration            1:Start calibration       0:Calibration finished    (0-Default)
	NAU7802_CAL_ERR             //Calibration Error            1:Calibration failed      0:No                      (0-Default)
	NAU7802_CRS0                //Converstion Rate samp/sec    1      0      1      0     1     0     1     0      (0-Default)
	NAU7802_CRS1                //Converstion Rate samp/sec    1 320  1 N/A  0 N/A  0 N/A 1 80  1 40  0 20  0 10   (0-Default)
	NAU7802_CRS2                //Converstion Rate samp/sec    1      1      1      1     0     0     0     0      (0-Default)
	NAU7802_CHS                 //Analog input channel         1:Channel 2               0:Channel 1               (0-Default)
)

//Allowed samples per second
const (
	NAU7802_SPS_10 byte = iota
	NAU7802_SPS_20
	NAU7802_SPS_40
	NAU7802_SPS_80
	NAU7802_SPS_320
)

// Allowed gain levels
const (
	NAU7802_GAIN_1 byte = iota
	NAU7802_GAIN_2
	NAU7802_GAIN_4
	NAU7802_GAIN_8
	NAU7802_GAIN_16
	NAU7802_GAIN_32
	NAU7802_GAIN_64
	NAU7802_GAIN_128
)

const (
	NAU7802_LDO_4V5 byte = iota
	NAU7802_LDO_4V2
	NAU7802_LDO_3V9
	NAU7802_LDO_3V6
	NAU7802_LDO_3V3
	NAU7802_LDO_3V0
	NAU7802_LDO_2V7
	NAU7802_LDO_2V4
)

type calibrationStatus int
const (
	NAU7802_CAL_SUCCESS calibrationStatus = iota
	NAU7802_CAL_IN_PROGRESS
	NAU7802_CAL_FAILURE
)
