package tijos.framework.sensor.max30100;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.util.logging.Logger;

//http://www.schwietering.com/jayduino/filtuino/
//Low pass butterworth filter order=1 alpha1=0.1
//Fs=100Hz, Fc=6Hz
class FilterBuLp1 {
	private double[] v = new double[2];

	public FilterBuLp1() {
		v[0] = 0.0;
	}

	public double step(double x) // class II
	{
		v[0] = v[1];
		v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * v[0]);
		return (v[0] + v[1]);
	}
};

// http://sam-koblenski.blogspot.de/2015/11/everyday-dsp-for-programmers-dc-and.html
class DCRemover {
	double alpha = 0;
	double dcw = 0;

	public DCRemover(double alpha_) {
		this.alpha = alpha_;
		this.dcw = 0;
	}

	public double step(double x) {
		double olddcw = dcw;
		dcw = (double) x + alpha * dcw;

		return dcw - olddcw;
	}

	public double getDCW() {
		return dcw;
	}

}

public class TiMAX30100_PulseOximeter {

	private static final int CURRENT_ADJUSTMENT_PERIOD_MS = 500;

	TiMAX30100 hrm;

	DCRemover irDCRemover;
	DCRemover redDCRemover;
	PulseOximeterState state = PulseOximeterState.PULSEOXIMETER_STATE_INIT;

	long tsFirstBeatDetected = 0;
	long tsLastBeatDetected = 0;
	long tsLastBiasCheck = 0;
	long tsLastCurrentAdjustment = 0;

	TiMAX30100_BeatDetector beatDetector = new TiMAX30100_BeatDetector();

	FilterBuLp1 lpf = new FilterBuLp1();
	int redLedCurrentIndex = TiMAX30100.MAX30100_LED_CURR_27_1MA;
	int irLedCurrent = TiMAX30100.MAX30100_LED_CURR_50MA;

	TiMAX30100_SpO2Calculator spO2calculator = new TiMAX30100_SpO2Calculator();

	private static final double DC_REMOVER_ALPHA = 0.95;

	enum PulseOximeterState {
		PULSEOXIMETER_STATE_INIT, PULSEOXIMETER_STATE_IDLE, PULSEOXIMETER_STATE_DETECTING
	};

	public TiMAX30100_PulseOximeter(TiI2CMaster i2c) {
		this.hrm = new TiMAX30100(i2c);
	}

	public void initialize() throws IOException {

		hrm.initialize();

		hrm.setMode(TiMAX30100.MAX30100_MODE_SPO2_HR);
		hrm.setLedsCurrent(irLedCurrent, redLedCurrentIndex);
		hrm.resetFifo();

		irDCRemover = new DCRemover(DC_REMOVER_ALPHA);
		redDCRemover = new DCRemover(DC_REMOVER_ALPHA);

		state = PulseOximeterState.PULSEOXIMETER_STATE_IDLE;
	}

	public void update() throws IOException {
		int sampleNum = hrm.update();

		if (sampleNum > 0) {
			checkSample(sampleNum);
			checkCurrentBias();
		}
	}

	public double getHeartRate() {
		return beatDetector.getRate();
	}

	public int getSpO2() {
		return spO2calculator.getSpO2();
	}

	public int getRedLedCurrentBias() {
		return redLedCurrentIndex;
	}

	public void setIRLedCurrent(int irLedNewCurrent) throws IOException {
		irLedCurrent = irLedNewCurrent;
		hrm.setLedsCurrent(irLedCurrent, redLedCurrentIndex);
	}

	public void shutdown() throws IOException {
		hrm.shutdown();
	}

	public void resume() throws IOException {
		hrm.resume();
	}

	private void checkSample(int sampleNum) {
		int rawIRValue, rawRedValue;

		// Dequeue all available samples, they're properly timed by the HRM
		while (sampleNum-- > 0) {

			rawIRValue = hrm.getIR();
			rawRedValue = hrm.getRed();

			double irACValue = irDCRemover.step(rawIRValue);
			double redACValue = redDCRemover.step(rawRedValue);

			// The signal fed to the beat detector is mirrored since the
			// cleanest monotonic spike is below zero
			double filteredPulseValue = lpf.step(-irACValue);
			boolean beatDetected = beatDetector.addSample(filteredPulseValue);

			if (beatDetector.getRate() > 0) {
				state = PulseOximeterState.PULSEOXIMETER_STATE_DETECTING;
				spO2calculator.update(irACValue, redACValue, beatDetected);
			} else if (state == PulseOximeterState.PULSEOXIMETER_STATE_DETECTING) {
				state = PulseOximeterState.PULSEOXIMETER_STATE_IDLE;
				spO2calculator.reset();
			}
		}
	}

	private void checkCurrentBias() throws IOException {
		// Follower that adjusts the red led current in order to have comparable
		// DC baselines between
		// red and IR leds. The numbers are really magic: the less possible to
		// avoid oscillations
		if (System.currentTimeMillis() - tsLastBiasCheck > CURRENT_ADJUSTMENT_PERIOD_MS) {
			boolean changed = false;
			if (irDCRemover.getDCW() - redDCRemover.getDCW() > 70000
					&& redLedCurrentIndex < TiMAX30100.MAX30100_LED_CURR_50MA) {
				++redLedCurrentIndex;
				changed = true;
			} else if (redDCRemover.getDCW() - irDCRemover.getDCW() > 70000 && redLedCurrentIndex > 0) {
				--redLedCurrentIndex;
				changed = true;
			}

			if (changed) {
				hrm.setLedsCurrent(irLedCurrent, redLedCurrentIndex);
				tsLastCurrentAdjustment = System.currentTimeMillis();
			}

			tsLastBiasCheck = System.currentTimeMillis();
		}
	}
}
