package us.ihmc.etherCAT.slaves;

import us.ihmc.etherCAT.slaves.EasyCATSlave;

import java.io.IOException;

public class EasyCATLoadCellSlave extends EasyCATSlave {

	private int[] loadCellRawValues = new int[4];
	private int[] loadCellProcessedValues = new int[4];
	private int[] loadCellCalibrationOffsetValues = new int[4];

	private int[] frameData = new int[32];
	private boolean calibrate = true;

	public EasyCATLoadCellSlave(int alias, int ringPosition) throws IOException {
		super(alias, ringPosition);
	}

	public void processLoadCellData() {

		getTransmitBytes(frameData, 0, 31);

		// 5 bytes per cell. 20 bytes for all 4 load cells. Maximum resolution of 16-bit
		// on the load cells at 15 SPS

		loadCellRawValues[0] = 10000 * frameData[1] + 1000 * frameData[2] + 100 * frameData[3] + 10 * frameData[4]
				+ frameData[5];
		loadCellRawValues[1] = 10000 * frameData[6] + 1000 * frameData[7] + 100 * frameData[8] + 10 * frameData[9]
				+ frameData[10];
		loadCellRawValues[2] = 10000 * frameData[11] + 1000 * frameData[12] + 100 * frameData[13] + 10 * frameData[14]
				+ frameData[15];
		loadCellRawValues[3] = 10000 * frameData[16] + 1000 * frameData[17] + 100 * frameData[18] + 10 * frameData[19]
				+ frameData[20];

		processRawData();
		
		limitData();


		if (calibrate)
			calibrate();

	}

	private void limitData() {
		// Limit to positive values
		if (loadCellProcessedValues[0] < 0.0) {
			loadCellProcessedValues[0] = 0;
		}

		if (loadCellProcessedValues[1] < 0.0) {
			loadCellProcessedValues[1] = 0;
		}

		if (loadCellProcessedValues[2] < 0.0) {
			loadCellProcessedValues[2] = 0;
		}

		if (loadCellProcessedValues[3] < 0.0) {
			loadCellProcessedValues[3] = 0;
		}
	}

	private void processRawData() {

		// TODO convert values into Newtons
		loadCellProcessedValues[0] = loadCellRawValues[0] - loadCellCalibrationOffsetValues[0];
		loadCellProcessedValues[1] = loadCellRawValues[1] - loadCellCalibrationOffsetValues[1];
		loadCellProcessedValues[2] = loadCellRawValues[2] - loadCellCalibrationOffsetValues[2];
		loadCellProcessedValues[3] = loadCellRawValues[3] - loadCellCalibrationOffsetValues[3];
	}

	private void calibrate() {

		// TODO calculate linear calibration for Newtons
		calibrate = false;

		loadCellCalibrationOffsetValues[0] = loadCellRawValues[0];
		loadCellCalibrationOffsetValues[1] = loadCellRawValues[1];
		loadCellCalibrationOffsetValues[2] = loadCellRawValues[2];
		loadCellCalibrationOffsetValues[3] = loadCellRawValues[3];
	}

	public void requestCalibration() {
		calibrate = true;
	}

	public int getLoadCell1RawValue() {
		return loadCellRawValues[0];
	}

	public int getLoadCell2RawValue() {
		return loadCellRawValues[1];
	}

	public int getLoadCell3RawValue() {
		return loadCellRawValues[2];
	}

	public int getLoadCell4RawValue() {
		return loadCellRawValues[3];
	}

	public int[] getLoadCellRawValues() {
		return loadCellRawValues;
	}

	public int getLoadCell1ProcessedValue() {
		return loadCellProcessedValues[0];
	}

	public int getLoadCell2ProcessedValue() {
		return loadCellProcessedValues[1];
	}

	public int getLoadCell3ProcessedValue() {
		return loadCellProcessedValues[2];
	}

	public int getLoadCell4ProcessedValue() {
		return loadCellProcessedValues[3];
	}

	public int[] getLoadCellProcessedValues() {
		return loadCellProcessedValues;
	}

}
