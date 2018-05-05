package tijos.framework.sensor.max30100;
 
public class TiMAX30100_BeatDetector {
	enum BeatDetectorState {
		BEATDETECTOR_STATE_INIT, BEATDETECTOR_STATE_WAITING, BEATDETECTOR_STATE_FOLLOWING_SLOPE, BEATDETECTOR_STATE_MAYBE_DETECTED, BEATDETECTOR_STATE_MASKING
	};

	// in ms, how long to wait before counting
	public static final int BEATDETECTOR_INIT_HOLDOFF = 2000;

	// in ms, non-retriggerable window after beat detection
	public static final int BEATDETECTOR_MASKING_HOLDOFF = 200;

	// EMA factor for the beat period value
	public static final double BEATDETECTOR_BPFILTER_ALPHA = 0.6;

	// maximum negative jump that triggers the beat edge
	public static final int BEATDETECTOR_STEP_RESILIENCY = 30;

	// minimum threshold (filtered) value
	public static final int BEATDETECTOR_MIN_THRESHOLD = 20;

	// maximum threshold (filtered) value
	public static final int BEATDETECTOR_MAX_THRESHOLD = 800;

	// thr chasing factor of the max value when beat
	public static final double BEATDETECTOR_THRESHOLD_FALLOFF_TARGET = 0.3;

	// thr chasing factor when no beat
	public static final double BEATDETECTOR_THRESHOLD_DECAY_FACTOR = 0.99;

	// in ms, no-beat time to cause a reset
	public static final int BEATDETECTOR_INVALID_READOUT_DELAY = 2000;

	// in ms, 1/Fs
	public static final int BEATDETECTOR_SAMPLES_PERIOD = 10;

	BeatDetectorState state = BeatDetectorState.BEATDETECTOR_STATE_INIT;
	double threshold = BEATDETECTOR_MIN_THRESHOLD;
	double beatPeriod = 0;
	double lastMaxValue = 0;
	long tsLastBeat = 0;

	public TiMAX30100_BeatDetector() {

	}

	public boolean addSample(double sample) {
		return checkForBeat(sample);
	}

	public double getRate() {
		if (beatPeriod != 0) {
			return 1 / beatPeriod * 1000 * 60;
		} else {
			return 0;
		}
	}

	public double getCurrentThreshold() {
		return threshold;
	}

	private boolean checkForBeat(double sample) {
		boolean beatDetected = false;
	
		switch (state) {
		case BEATDETECTOR_STATE_INIT:
			if (System.currentTimeMillis() > BEATDETECTOR_INIT_HOLDOFF) {
				state = BeatDetectorState.BEATDETECTOR_STATE_WAITING;
			}
			break;

		case BEATDETECTOR_STATE_WAITING:
			if (sample > threshold) {
				threshold = Math.min(sample, BEATDETECTOR_MAX_THRESHOLD);
				state = BeatDetectorState.BEATDETECTOR_STATE_FOLLOWING_SLOPE;
			}

			// Tracking lost, resetting
			if (System.currentTimeMillis() - tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY) {
				beatPeriod = 0;
				lastMaxValue = 0;
			}

			decreaseThreshold();
			break;

		case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
			if (sample < threshold) {
				state = BeatDetectorState.BEATDETECTOR_STATE_MAYBE_DETECTED;
			} else {
				threshold = Math.min(sample, BEATDETECTOR_MAX_THRESHOLD);
			}
			break;

		case BEATDETECTOR_STATE_MAYBE_DETECTED:
			if (sample + BEATDETECTOR_STEP_RESILIENCY < threshold) {
				// Found a beat
				beatDetected = true;
				lastMaxValue = sample;
				state = BeatDetectorState.BEATDETECTOR_STATE_MASKING;

				long delta = System.currentTimeMillis() - tsLastBeat;
				if (delta > 0) {
					beatPeriod = BEATDETECTOR_BPFILTER_ALPHA * delta + (1 - BEATDETECTOR_BPFILTER_ALPHA) * beatPeriod;
				}

				tsLastBeat = System.currentTimeMillis();
			} else {
				state = BeatDetectorState.BEATDETECTOR_STATE_FOLLOWING_SLOPE;
			}
			break;

		case BEATDETECTOR_STATE_MASKING:
			if (System.currentTimeMillis() - tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF) {
				state = BeatDetectorState.BEATDETECTOR_STATE_WAITING;
			}
			decreaseThreshold();
			break;
		}

		return beatDetected;
	}

	private void decreaseThreshold() {
		// When a valid beat rate readout is present, target the
		if (lastMaxValue > 0 && beatPeriod > 0) {
			threshold -= lastMaxValue * (1 - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET)
					/ (beatPeriod / BEATDETECTOR_SAMPLES_PERIOD);
		} else {
			// Asymptotic decay
			threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
		}

		if (threshold < BEATDETECTOR_MIN_THRESHOLD) {
			threshold = BEATDETECTOR_MIN_THRESHOLD;
		}
	}

}
