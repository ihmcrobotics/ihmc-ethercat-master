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
		
		//4 bytes per cell. 16 bytes for all 4 load cells

		loadCell1Raw = 1000 * frameData[2] + 100 * frameData[3] + 10 * frameData[4] + frameData[5];
		loadCell2Raw = 1000 * frameData[6] + 100 * frameData[7] + 10 * frameData[8] + frameData[9];
		loadCell3Raw = 1000 * frameData[10] + 100 * frameData[11] + 10 * frameData[12] + frameData[13];
		loadCell4Raw = 1000 * frameData[14] + 100 * frameData[15] + 10 * frameData[16] + frameData[17];

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
