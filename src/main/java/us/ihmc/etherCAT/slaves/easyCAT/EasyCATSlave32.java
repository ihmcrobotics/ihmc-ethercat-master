package us.ihmc.etherCAT.slaves.easyCAT;

import java.io.IOException;

public class EasyCATSlave32 extends EasyCATSlave
{
	static final long vendorID = 0x0000079a;
	static final long productCode = 0x00DEFEDEL;
	static final int easyCATSlave32FrameLength = 32;
	public EasyCATSlave32(int alias, int ringPosition) throws IOException
	{
		super(vendorID, productCode, easyCATSlave32FrameLength, alias, ringPosition);
	}
	
}