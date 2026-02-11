package us.ihmc.etherCAT.slaves.easyCAT;

import java.io.IOException;

public class EasyCATSlave64 extends EasyCATSlave
{
	static final long vendorID = 0x0000079a;
	static final long productCode = 0xDEFEDE64L;
	static final int easyCATSlave64FrameLength = 64;
	public EasyCATSlave64(int alias, int ringPosition) throws IOException
	{
		super(vendorID, productCode, easyCATSlave64FrameLength, alias, ringPosition);
	}
	
}