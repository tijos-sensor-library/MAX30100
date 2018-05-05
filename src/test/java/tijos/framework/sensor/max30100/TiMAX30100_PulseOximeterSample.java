package tijos.framework.sensor.max30100;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.sensor.max30100.TiMAX30100_PulseOximeter;
import tijos.framework.util.Delay;

public class TiMAX30100_PulseOximeterSample {

	public static void main(String[] args) {
		/*
		 * 定义使用的TiI2CMaster port
		 */
		int i2cPort0 = 0;

		/*
		 * 资源分配， 将i2cPort0分配给TiI2CMaster对象i2c0
		 */
		TiI2CMaster i2c0 = null;
		try {
			i2c0 = TiI2CMaster.open(i2cPort0);
		} catch (IOException e) {
		
			e.printStackTrace();
		}
		
		
		TiMAX30100_PulseOximeter pox = new TiMAX30100_PulseOximeter(i2c0);

		try {
			// Initialize the PulseOximeter instance
			// Failures are generally due to an improper I2C wiring, missing
			// power supply or wrong target chip
			pox.initialize();

			while (true) {
				// Make sure to call update as fast as possible
				pox.update();
				// Asynchronously dump heart rate and oxidation levels to the serial 
				//For both, a value of 0 means "invalid"
				double hr = pox.getHeartRate();
				if(hr > 1)
					System.out.println("heart " + (int)hr);

				double spO2 = pox.getSpO2();
				if(spO2 > 0)
					System.out.println("spO2 " + (int)spO2);

			}

		} catch (IOException ex) {
			ex.printStackTrace();
		}

	}

}
