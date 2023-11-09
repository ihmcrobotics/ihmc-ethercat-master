package us.ihmc.etherCAT.slaves.easyCAT;

import java.io.IOException;

public class EasyCATSlave128 extends EasyCATSlave
{
	static final long vendorID = 0x0000079a;
	static final long productCode = 0xDEFED128L;
	static final int easyCATSlave128FrameLength = 128;
	public EasyCATSlave128(int alias, int ringPosition) throws IOException
	{
		super(vendorID, productCode, easyCATSlave128FrameLength, alias, ringPosition);
	}
	
}