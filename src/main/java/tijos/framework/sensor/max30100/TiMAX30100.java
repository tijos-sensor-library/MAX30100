

package tijos.framework.sensor.max30100;

import java.io.IOException;
import java.util.ArrayDeque;
import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.util.BigBitConverter;

/* TiJOS-MAX30100 oximetry / heart rate integrated sensor library
 * Based on original C library from Ardino by oxullo  
 * https://github.com/oxullo/Arduino-MAX30100
 */

class TiMAX30100Regsiters {

	// Interrupt status register (RO)
	public static final int MAX30100_REG_INTERRUPT_STATUS = 0x00;

	public static final int MAX30100_IS_PWR_RDY = (1 << 0);
	public static final int MAX30100_IS_SPO2_RDY = (1 << 4);
	public static final int MAX30100_IS_HR_RDY = (1 << 5);
	public static final int MAX30100_IS_TEMP_RDY = (1 << 6);
	public static final int MAX30100_IS_A_FULL = (1 << 7);

	// Interrupt enable register
	public static final int MAX30100_REG_INTERRUPT_ENABLE = 0x01;
	public static final int MAX30100_IE_ENB_SPO2_RDY = (1 << 4);
	public static final int MAX30100_IE_ENB_HR_RDY = (1 << 5);
	public static final int MAX30100_IE_ENB_TEMP_RDY = (1 << 6);
	public static final int MAX30100_IE_ENB_A_FULL = (1 << 7);

	// FIFO control and data registers
	public static final int MAX30100_REG_FIFO_WRITE_POINTER = 0x02;
	public static final int MAX30100_REG_FIFO_OVERFLOW_COUNTER = 0x03;
	public static final int MAX30100_REG_FIFO_READ_POINTER = 0x04;
	
	// Burst read does not auto increment addr
	public static final int MAX30100_REG_FIFO_DATA = 0x05; 

	// Mode Configuration register
	public static final int MAX30100_REG_MODE_CONFIGURATION = 0x06;

	// SpO2 Configuration register
	// Check tables 8 and 9, p19 of the MAX30100 datasheet to see the
	// permissible
	// combinations of sampling rates and pulse widths
	public static final int MAX30100_REG_SPO2_CONFIGURATION = 0x07;

	// LED Configuration register
	public static final int MAX30100_REG_LED_CONFIGURATION = 0x09;

	// Temperature integer part register
	public static final int MAX30100_REG_TEMPERATURE_DATA_INT = 0x16;
	// Temperature fractional part register
	public static final int MAX30100_REG_TEMPERATURE_DATA_FRAC = 0x17;

	// Revision ID register (RO)
	public static final int MAX30100_REG_REVISION_ID = 0xfe;
	// Part ID register
	public static final int MAX30100_REG_PART_ID = 0xff;

}

/**
 * MAX30100 oximetry / heart rate integrated sensor
 * 
 * @author TiJOS
 *
 */
public class TiMAX30100 {

	/**
	 * I2C slave address
	 */
	public static final int MAX30100_SLAVE_ADDRESS = 0x57;

	/**
	 * heart-rate only mode
	 */
	public static final int MAX30100_MODE_HRONLY = 0x02;

	/**
	 * SpO2 and heart-rate modes
	 */
	public static final int MAX30100_MODE_SPO2_HR = 0x03;

	/**
	 * LED Current
	 */
	public static final int MAX30100_LED_CURR_0MA = 0x00;
	public static final int MAX30100_LED_CURR_4_4MA = 0x01;
	public static final int MAX30100_LED_CURR_7_6MA = 0x02;
	public static final int MAX30100_LED_CURR_11MA = 0x03;
	public static final int MAX30100_LED_CURR_14_2MA = 0x04;
	public static final int MAX30100_LED_CURR_17_4MA = 0x05;
	public static final int MAX30100_LED_CURR_20_8MA = 0x06;
	public static final int MAX30100_LED_CURR_24MA = 0x07;
	public static final int MAX30100_LED_CURR_27_1MA = 0x08;
	public static final int MAX30100_LED_CURR_30_6MA = 0x09;

	public static final int MAX30100_LED_CURR_33_8MA = 0x0a;
	public static final int MAX30100_LED_CURR_37MA = 0x0b;
	public static final int MAX30100_LED_CURR_40_2MA = 0x0c;
	public static final int MAX30100_LED_CURR_43_6MA = 0x0d;
	public static final int MAX30100_LED_CURR_46_8MA = 0x0e;
	public static final int MAX30100_LED_CURR_50MA = 0x0f;

	/**
	 * LED Pulse width
	 */
	public static final int MAX30100_SPC_PW_200US_13BITS = 0x00;
	public static final int MAX30100_SPC_PW_400US_14BITS = 0x01;
	public static final int MAX30100_SPC_PW_800US_15BITS = 0x02;
	public static final int MAX30100_SPC_PW_1600US_16BITS = 0x03;

	/**
	 * Sampling Rate
	 */
	public static final int MAX30100_SAMPRATE_50HZ = 0x00;
	public static final int MAX30100_SAMPRATE_100HZ = 0x01;
	public static final int MAX30100_SAMPRATE_167HZ = 0x02;
	public static final int MAX30100_SAMPRATE_200HZ = 0x03;
	public static final int MAX30100_SAMPRATE_400HZ = 0x04;
	public static final int MAX30100_SAMPRATE_600HZ = 0x05;
	public static final int MAX30100_SAMPRATE_800HZ = 0x06;
	public static final int MAX30100_SAMPRATE_1000HZ = 0x07;

	/**
	 * Mode configuration
	 */
	public static final int MAX30100_MC_TEMP_EN = (1 << 3);
	public static final int MAX30100_MC_RESET = (1 << 6);
	public static final int MAX30100_MC_SHDN = (1 << 7);

	/**
	 * SpO2 Configuration
	 */
	public static final int MAX30100_SPC_SPO2_HI_RES_EN = (1 << 6);

	public static final int MAX30100_FIFO_DEPTH = 0x10;
	
	public static final int  EXPECTED_PART_ID   = 0x11;

	private TiI2CMaster i2cmObj;

	private ArrayDeque<Integer> rawIRQueue = new ArrayDeque<Integer>();
	private ArrayDeque<Integer> rawRedQueue = new ArrayDeque<Integer>();

	private byte[] buffer = new byte[MAX30100_FIFO_DEPTH * 4];

	/**
	 * I2C slave address
	 */
	private int i2cSlaveAddr = MAX30100_SLAVE_ADDRESS;

	private byte[] data = new byte[4];

	/**
	 * Initialize with default I2C address 0xAE
	 * 
	 * @param i2c
	 */
	public TiMAX30100(TiI2CMaster i2c) {
		this(i2c, MAX30100_SLAVE_ADDRESS);
	}

	/**
	 * Initialize with I2C and Slave address
	 * 
	 * @param i2c
	 * @param address
	 */
	public TiMAX30100(TiI2CMaster i2c, int address) {
		this.i2cmObj = i2c;
		this.i2cSlaveAddr = address;
	}

	/**
	 * Initialize with default settings
	 * 
	 * @throws IOException
	 */
	public void initialize() throws IOException {

		this.i2cmObj.setWorkBaudrate(400);
		
		int partId = getPartId();
		
		if (partId != EXPECTED_PART_ID) {
		    throw new IOException("partId is not expected. " + partId);
		}
		
		setMode(MAX30100_MODE_HRONLY);
		setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);
		setSamplingRate(MAX30100_SAMPRATE_100HZ);
		setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_50MA);
		setHighresModeEnabled(true);
	}

	/**
	 * Get sample values of FIFO DATA
	 * 
	 * @throws IOException
	 */
	public int update() throws IOException {

		return readFifoData();
	}

	/**
	 * return IR value of measure result
	 * 
	 * @return
	 */
	public int getIR() {
		return this.rawIRQueue.pop();
	}

	/**
	 * return Red value of measure result
	 * 
	 * @return
	 */
	public int getRed() {
		return this.rawRedQueue.pop();
	}

	/**
	 * Mode Control
	 * 
	 * @param mode
	 *            MAX30100_MODE_HRONLY or MAX30100_MODE_SPO2_HR
	 * @throws IOException
	 */
	public void setMode(int mode) throws IOException {

		data[0] = (byte) mode;
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_MODE_CONFIGURATION, data, 0, 1);
	}

	/**
	 * LED Pulse Width Control
	 * 
	 * @param ledPulseWidth
	 * @throws IOException
	 */
	public void setLedsPulseWidth(int ledPulseWidth) throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_SPO2_CONFIGURATION, data, 0, 1);
		int previous = data[0] & 0xFF;

		int width = (previous & 0xfc) | ledPulseWidth;

		data[0] = (byte) width;
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_SPO2_CONFIGURATION, data, 0, 1);
	}

	/**
	 * SpO2 Sample Rate Control
	 * 
	 * @param samplingRate
	 * @throws IOException
	 */
	public void setSamplingRate(int samplingRate) throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_SPO2_CONFIGURATION, data, 0, 1);
		int previous = data[0] & 0xFF;

		int rate = (previous & 0xe3) | (samplingRate << 2);

		data[0] = (byte) rate;

		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_SPO2_CONFIGURATION, data, 0, 1);
	}

	/**
	 * LED Current Control
	 * 
	 * @param irLedCurrent
	 *            the current level of the IR LED
	 * @param redLedCurrent
	 *            the current level of the Red LED
	 * @throws IOException
	 */
	public void setLedsCurrent(int irLedCurrent, int redLedCurrent) throws IOException {

		data[0] = (byte) (redLedCurrent << 4 | irLedCurrent);
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_LED_CONFIGURATION, data, 0, 1);
	}

	/**
	 * SpO2 High Resolution Enable
	 * 
	 * @param enabled
	 * @throws IOException
	 */
	public void setHighresModeEnabled(boolean enabled) throws IOException {

		this.i2cmObj.read(this.i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_SPO2_CONFIGURATION, data, 0, 1);
		int previous = data[0] & 0xFF;

		int newValue = 0;
		if (enabled) {
			newValue = previous | MAX30100_SPC_SPO2_HI_RES_EN;
		} else {
			newValue = previous & ~MAX30100_SPC_SPO2_HI_RES_EN;
		}

		data[0] = (byte) newValue;
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_SPO2_CONFIGURATION, data, 0, 1);
	}

	/**
	 * This is a self-clearing bit which, when set, initiates a single
	 * temperature reading from the temperature sensor. This bit is cleared
	 * automatically back to zero at the conclusion of the temperature reading
	 * when the bit is set to one in heart rate or SpO2 mode.
	 *
	 * @throws IOException
	 */
	public void startTemperatureSampling() throws IOException {

		this.i2cmObj.read(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_MODE_CONFIGURATION, data, 0, 1);
		int modeConfig = data[0] & 0xFF;

		modeConfig |= MAX30100_MC_TEMP_EN;

		data[0] = (byte) modeConfig;
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_MODE_CONFIGURATION, data, 0, 1);
	}

	/**
	 * is temperature data ready
	 * 
	 * @return
	 * @throws IOException
	 */
	public boolean isTemperatureReady() throws IOException {

		this.i2cmObj.read(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_MODE_CONFIGURATION, data, 0, 1);
		if (((data[0] & 0xFF) & MAX30100_MC_TEMP_EN) > 0)
			return true;

		return false;
	}

	/**
	 * The on-board temperature ADC output is split into two registers, one to
	 * store the integer temperature and one to store the fraction. Both should
	 * be read when reading the temperature data, and the following equation
	 * shows how to add the two registers together: TMEASURED = TINTEGER +
	 * TFRACTION This register stores the integer temperature data in two’s
	 * complement format, where each bit corresponds to degree Celsius.
	 *
	 * @return temperature data
	 * @throws IOException
	 */
	public double retrieveTemperature() throws IOException {

		this.i2cmObj.read(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_TEMPERATURE_DATA_INT, data, 0, 1);
		int tempInteger = data[0] & 0xFF;

		this.i2cmObj.read(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_TEMPERATURE_DATA_FRAC, data, 0, 1);
		int tempFrac = data[0] & 0xFF;

		return tempFrac * 0.0625 + tempInteger;
	}

	/**
	 * The part can be put into a power-save mode by setting this bit to one.
	 * While in power-save mode, all registers retain their values, and
	 * write/read operations function as normal. All interrupts are cleared to
	 * zero in this mode.
	 * 
	 * @throws IOException
	 */
	public void shutdown() throws IOException {

		this.i2cmObj.read(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_MODE_CONFIGURATION, data, 0, 1);
		int modeConfig = data[0] & 0xFF;

		modeConfig |= MAX30100_MC_SHDN;

		data[0] = (byte) modeConfig;
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_MODE_CONFIGURATION, data, 0, 1);

	}

	/**
	 * 
	 * @throws IOException
	 */
	public void resume() throws IOException {

		this.i2cmObj.read(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_MODE_CONFIGURATION, data, 0, 1);
		int modeConfig = data[0] & 0xFF;

		modeConfig &= ~MAX30100_MC_SHDN;

		data[0] = (byte) modeConfig;
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_MODE_CONFIGURATION, data, 0, 1);
	}

	/**
	 * get device part id
	 * 
	 * @return part id
	 * @throws IOException
	 */
	public int getPartId() throws IOException {
		this.i2cmObj.read(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_PART_ID, data, 0, 1);
		return data[0] & 0xFF;
	}

	/**
	 * Reset FIFO
	 * 
	 * @throws IOException
	 */
	public void resetFifo() throws IOException {
		data[0] = 0;
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_FIFO_WRITE_POINTER, data, 0, 1);
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_FIFO_READ_POINTER, data, 0, 1);
		this.i2cmObj.write(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_FIFO_OVERFLOW_COUNTER, data, 0, 1);

	}

	private int readFifoData() throws IOException {
		
		this.i2cmObj.read(i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_FIFO_WRITE_POINTER, data, 0, 3);
		int writePointer = data[0] & 0xFF;
		int overflow = data[1] & 0xFF;
		int readPointer = data[2] & 0xFF;
		int toRead = (writePointer - readPointer) & (MAX30100_FIFO_DEPTH - 1);
		
		//if overflow, read max depth data 
		if(overflow > 0)
		{
			toRead = MAX30100_FIFO_DEPTH;
		}
		
		if (toRead > 0) {
			this.i2cmObj.read(this.i2cSlaveAddr, TiMAX30100Regsiters.MAX30100_REG_FIFO_DATA, buffer, 0, 4 * toRead);

			for (int i = 0; i < toRead; i++) {
				// Warning: the values are always left-aligned
				int rawIRValue = BigBitConverter.ToUInt16(buffer, i * 4);
				int rawRedValue = BigBitConverter.ToUInt16(buffer, i * 4 + 2);

				this.rawIRQueue.add(rawIRValue);
				this.rawRedQueue.add(rawRedValue);

			}
		}
		
		return rawIRQueue.size();

	}

}
