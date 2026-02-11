package us.ihmc.etherCAT.slaves.easyCAT;

import java.io.IOException;

public class EasyCATSlave16 extends EasyCATSlave
{
	static final long vendorID = 0x0000079a;
	static final long productCode = 0xDEFEDE16L;
	static final int easyCATSlave16FrameLength = 16;
	public EasyCATSlave16(int alias, int ringPosition) throws IOException
	{
		super(vendorID, productCode, easyCATSlave16FrameLength, alias, ringPosition);
	}
	
}