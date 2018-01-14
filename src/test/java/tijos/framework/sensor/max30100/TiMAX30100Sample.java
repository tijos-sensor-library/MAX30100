package tijos.framework.sensor.max30100;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.sensor.max30100.TiMAX30100;

public class TiMAX30100Sample {

	public static void main(String[] args) {
		try {
			/*
			 * 定义使用的TiI2CMaster port
			 */
			int i2cPort0 = 0;

			/*
			 * 资源分配， 将i2cPort0分配给TiI2CMaster对象i2c0
			 */
			TiI2CMaster i2c0 = TiI2CMaster.open(i2cPort0);

			TiMAX30100 sensor = new TiMAX30100(i2c0);
			sensor.initialize();
			
		    // Set up the wanted parameters
		    sensor.setMode(TiMAX30100.MAX30100_MODE_SPO2_HR);
		    sensor.setLedsCurrent(TiMAX30100.MAX30100_LED_CURR_50MA, TiMAX30100.MAX30100_LED_CURR_27_1MA);
		    sensor.setLedsPulseWidth(TiMAX30100.MAX30100_SPC_PW_1600US_16BITS);
		    sensor.setSamplingRate(TiMAX30100.MAX30100_SAMPRATE_100HZ);
		    sensor.setHighresModeEnabled(true);
		    sensor.resetFifo();

			while (true) {
				try {
					
					int sampleNumber = sensor.update();
					while(sampleNumber-- > 0){
						System.out.println("IR=  "  + sensor.getIR());
						System.out.println("Red= "  + sensor.getRed());
					}
					
				} catch (Exception ex) {

					ex.printStackTrace();
				}

			}
		} catch (IOException ie) {
			ie.printStackTrace();
		}

	}

}
