package us.ihmc.etherCAT.slaves;

import java.io.IOException;

public class EasyCATLoadCellSlave extends EasyCATSlave {

	private int loadCell1Raw;
	private int loadCell2Raw;
	private int loadCell3Raw;
	private int loadCell4Raw;
	private int[] loadCellRawValues = new int[4];

	private int[] frameData = new int[32];

	public EasyCATLoadCellSlave(int alias, int ringPosition) throws IOException {
		super(alias, ringPosition);
	}

	public void processLoadCellData() {

		getTransmitBytes(frameData, 0, 31);
		
		//5 bytes per cell. 20 bytes for all 4 load cells. Maximum resolution of 16-bit on the load cells at 15 SPS

		loadCell1Raw = 10000 * frameData[1] + 1000 * frameData[2] + 100 * frameData[3] + 10 * frameData[4] + frameData[5];
		loadCell2Raw = 10000 * frameData[6] +1000 * frameData[7] + 100 * frameData[8] + 10 * frameData[9] + frameData[10];
		loadCell3Raw = 10000 * frameData[11] +1000 * frameData[12] + 100 * frameData[13] + 10 * frameData[14] + frameData[15];
		loadCell4Raw = 10000 * frameData[16] +1000 * frameData[17] + 100 * frameData[18] + 10 * frameData[19] + frameData[20];

		loadCellRawValues = new int[] { loadCell1Raw, loadCell2Raw, loadCell3Raw, loadCell4Raw };
	}

	public int getLoadCell1RawValue() {
		return loadCell1Raw;
	}

	public int getLoadCell2RawValue() {
		return loadCell2Raw;
	}

	public int getLoadCell3RawValue() {
		return loadCell3Raw;
	}

	public int getLoadCell4RawValue() {
		return loadCell4Raw;
	}

	public int[] getLoadCellRawValues() {
		return loadCellRawValues;
	}

}
