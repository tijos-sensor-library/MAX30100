package tijos.framework.sensor.max30100;

public class TiMAX30100_SpO2Calculator {

	private static final int CALCULATE_EVERY_N_BEATS = 3;

	// SaO2 Look-up Table
	// http://www.ti.com/lit/an/slaa274b/slaa274b.pdf
	static byte[] spO2LUT = new byte[] {100,100,100,100,
			99,99,99,99,99,99,
			98,98,98,98,98,
			97,97,97,97,97,97,
			96,96,96,96,96,96,
			95,95,95,95,95,95,
			94,94,94,94,94,
			93,93,93,93,93};

	double irACValueSqSum = 0;
	double redACValueSqSum = 0;
	int beatsDetectedNum = 0;
	long samplesRecorded = 0;
	int spO2 = 0;

	public TiMAX30100_SpO2Calculator() {

	}

	public void update(double irACValue, double redACValue, boolean beatDetected) {
		irACValueSqSum += irACValue * irACValue;
		redACValueSqSum += redACValue * redACValue;
		++samplesRecorded;

		if (beatDetected) {
			++beatsDetectedNum;
			if (beatsDetectedNum == CALCULATE_EVERY_N_BEATS) {
				double acSqRatio = Math.log(redACValueSqSum / samplesRecorded)
						/ Math.log(irACValueSqSum / samplesRecorded);
				
				acSqRatio *= 100.0;
				int index = 0;

				if (acSqRatio > 66) {
					index = (int) acSqRatio - 66;
				} else if (acSqRatio > 50) {
					index = (int) acSqRatio - 50;
				}
				reset();

				if(index < spO2LUT.length)
					spO2 = spO2LUT[index];
			}
		}
	}

	public void reset() {
		samplesRecorded = 0;
		redACValueSqSum = 0;
		irACValueSqSum = 0;
		beatsDetectedNum = 0;
		spO2 = 0;
	}

	public int getSpO2() {
		return spO2;
	}

}
