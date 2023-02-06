package us.ihmc.etherCAT.slaves.omron;

import us.ihmc.etherCAT.slaves.DSP402Slave;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.TxPDO;

/**
 * class: OmronR88D1SN
 * 
 * A motor controller which is communicated with via
 * the EtherCAT protocol. Extend this abstract class
 * for your particular implementation.
 * @author Nick Tremaroli (nicktrem@vt.edu)
 * @author An-Chi He (anchihe@vt.edu)
 */
public abstract class OmronR88D1SN extends DSP402Slave
{
	static final int vendorID = 0x00000083;
	static final int productCode = 0x000000b4;

	// PDO MAPPING 1 TXPDO
	protected class TPDO_1b01 extends TxPDO
	{
		protected TPDO_1b01()
		{
			super(0x1b01);
		}
		Unsigned16 errorCode = new Unsigned16();
		Unsigned16 statusWord = new Unsigned16();
		Signed32 positionActualValue = new Signed32();
		Signed16 torqueActualValue = new Signed16();
		Signed32 followingErrorActualValue = new Signed32();
		Unsigned16 touchProbeStatus = new Unsigned16();
		Signed32 touchProbe1PositiveEdge = new Signed32();
		Signed32 touchProbe2PositiveEdge = new Signed32();
		Unsigned32 digitalInputs = new Unsigned32();
	}
	
	// PDO MAPPING 1 RXPDO
	protected class RPDO_1701 extends RxPDO
	{
		protected RPDO_1701()
		{
			super(0x1701);
		}
		Unsigned16 controlWord = new Unsigned16();
		Signed32 targetPosition = new Signed32();
		Unsigned16 touchProbeFunciton = new Unsigned16();
		Unsigned32 physicalOutputs = new Unsigned32();
	}
	
	// PDO MAPPING 2 and PDO MAPPING 4 TXPDO
	protected class TPDO_1b02 extends TxPDO
	{
		protected TPDO_1b02()
		{
			super(0x1b02);
		}
		Unsigned16 errorCode = new Unsigned16();
		Unsigned16 statusWord = new Unsigned16();
		Signed32 positionActualValue = new Signed32();
		Signed16 torqueActualValue = new Signed16();
		Signed8 modesOfOperationDisplay = new Signed8();
		Unsigned16 touchProbeStatus = new Unsigned16();
		Signed32 touchProbe1PositiveEdge = new Signed32();
		Signed32 touchProbe2PositiveEdge = new Signed32();
		Unsigned32 digitalInputs = new Unsigned32();
	}
	
	// PDO MAPPING 2 RXPDO
	protected class RPDO_1702 extends RxPDO
	{
		protected RPDO_1702()
		{
			super(0x1702);
		}
		Unsigned16 controlWord = new Unsigned16();
		Signed32 targetPosition = new Signed32();
		Signed32 targetVelocity = new Signed32();
		Signed16 targetTorque = new Signed16();
		Signed8 modesOfOperation = new Signed8();
		Unsigned16 touchProbeFunciton = new Unsigned16();
		Unsigned32 maxProfileVelocity = new Unsigned32();
	}
	
	// PDO MAPPING 3 TXPDO
	protected class TPDO_1b03 extends TxPDO
	{
		protected TPDO_1b03()
		{
			super(0x1b03);
		}
		Unsigned16 errorCode = new Unsigned16();
		Unsigned16 statusWord = new Unsigned16();
		Signed32 positionActualValue = new Signed32();
		Signed16 torqueActualValue = new Signed16();
		Signed32 followingErrorActualValue = new Signed32();
		Signed8 modesOfOperationDisplay = new Signed8();
		Unsigned16 touchProbeStatus = new Unsigned16();
		Signed32 touchProbe1PositiveEdge = new Signed32();
		Signed32 touchProbe2PositiveEdge = new Signed32();
		Unsigned32 digitalInputs = new Unsigned32();
	}
	
	// PDO MAPPING 3 RXPDO
	protected class RPDO_1703 extends RxPDO
	{
		protected RPDO_1703()
		{
			super(0x1703);
		}
		Unsigned16 controlWord = new Unsigned16();
		Signed32 targetPosition = new Signed32();
		Signed32 targetVelocity = new Signed32();
		Signed8 modesOfOperation = new Signed8();
		Unsigned16 touchProbeFunciton = new Unsigned16();
		Unsigned16 positiveTorqueLimit = new Unsigned16();
		Unsigned16 negativeTorqueLimit = new Unsigned16();
	}
	
	// PDO MAPPING 4 RXPDO
	protected class RPDO_1704 extends RxPDO
	{
		protected RPDO_1704()
		{
			super(0x1704);
		}
		Unsigned16 controlWord = new Unsigned16();
		Signed32 targetPosition = new Signed32();
		Signed32 targetVelocity = new Signed32();
		Signed16 targetTorque = new Signed16();
		Signed8 modesOfOperation = new Signed8();
		Unsigned16 touchProbeFunciton = new Unsigned16();
		Unsigned32 maxProfileVelocity = new Unsigned32();
		Unsigned16 positiveTorqueLimit = new Unsigned16();
		Unsigned16 negativeTorqueLimit = new Unsigned16();
	}
	
	// PDO MAPPING 5 TXPDO
	protected class TPDO_1b04 extends TxPDO
	{
		protected TPDO_1b04()
		{
			super(0x1b04);
		}
		Unsigned16 errorCode = new Unsigned16();
		Unsigned16 statusWord = new Unsigned16();
		Signed32 positionActualValue = new Signed32();
		Signed16 torqueActualValue = new Signed16();
		Signed8 modesOfOperationDisplay = new Signed8();
		Unsigned16 touchProbeStatus = new Unsigned16();
		Signed32 touchProbe1PositiveEdge = new Signed32();
		Signed32 touchProbe2PositiveEdge = new Signed32();
		Unsigned32 digitalInputs = new Unsigned32();
		Signed32 velocityActualValue = new Signed32();
		
	}
	
	// PDO MAPPING 5 RXPDO
	protected class RPDO_1705 extends RxPDO
	{
		protected RPDO_1705()
		{
			super(0x1705);
		}
		Unsigned16 controlWord = new Unsigned16();
		Signed32 targetPosition = new Signed32();
		Signed32 targetVelocity = new Signed32();
		Signed8 modesOfOperation = new Signed8();
		Unsigned16 touchProbeFunciton = new Unsigned16();
		Unsigned16 positiveTorqueLimit = new Unsigned16();
		Unsigned16 negativeTorqueLimit = new Unsigned16();
		Signed16 torqueOffset = new Signed16();		
	}
	
	// VARIABLE PDO MAPPING TXPDO
	// should you want to use the variable PDO mapping
	// extend this abstract class in your implementation and
	// pack the PDO how you see fit
	protected abstract class TPDO_1a00 extends TxPDO
	{
		protected TPDO_1a00()
		{
			super(0x1A00);
		}
	}

	// VARIABLE PDO MAPPING RXPDO
	// should you want to use the variable PDO mapping
	// extend this abstract class in your implementation and
	// pack the PDO how you see fit
	protected abstract class RPDO_1600 extends RxPDO
	{
		protected RPDO_1600()
		{
			super(0x1600);
		}
	}

	/**
	 * Method: constructor
	 * 
	 * creates and initializes the Omron motor controller for EtherCAT communication
	 * @param alias: the alias number of the drive
	 * @param ringPosition: the ring position of the drive
	 */
	public OmronR88D1SN(int alias, int ringPosition)
	{
		super(vendorID, productCode, alias, ringPosition);
	}
}
