package us.ihmc.etherCAT.slaves.elmo;

import java.io.IOException;

import us.ihmc.etherCAT.javalution.Struct.Unsigned16;
import us.ihmc.etherCAT.master.SyncManager;

public class ElmoTwitterLLACarrierBoard extends ElmoTwitter
{
   static final int vendorID = 0x0000009a;
   static final int productCode = 0x00030924;
   static final int assignActivate = 0x0300;

   //Receive PDOs
   private final RPDO_1606 rpdo_1606 = new RPDO_1606();
   private final RPDO_160B rpdo_160B = new RPDO_160B();


   //Transmit PDOs
   private final TPDO_1a03 TPDO_1a03 = new TPDO_1a03();
   private final TPDO_1a12 TPDO_1a12 = new TPDO_1a12();
   private final TPDO_1a13 TPDO_1a13 = new TPDO_1a13();
   private final TPDO_1a18 TPDO_1a18 = new TPDO_1a18();
   private final TPDO_1a1d TPDO_1a1d = new TPDO_1a1d();
   private final TPDO_1a1e TPDO_1a1e = new TPDO_1a1e();
   private final TPDO_1a22 TPDO_1a22 = new TPDO_1a22();

   //Max Torque
   private long maxDriveTorque;

   //Digital Outputs
   private long digitalOutputBitString = 0l;

   public ElmoTwitterLLACarrierBoard(int alias, int ringPosition) throws IOException
   {
      super(alias, ringPosition);

      //Sync Manager, only SM2 and SM3 are used for PDOs.  SM0 and SM1 are used for the mailbox mechanism (See: EtherCAT_Application_Manual.pdf)

      for (int i = 2; i < 4; i++)
      {
         registerSyncManager(new SyncManager(i, true));
      }

      sm(2).registerPDO(rpdo_1606);
      sm(2).registerPDO(rpdo_160B);

      sm(3).registerPDO(TPDO_1a03);
      sm(3).registerPDO(TPDO_1a12);
      sm(3).registerPDO(TPDO_1a13);
      sm(3).registerPDO(TPDO_1a18);
      sm(3).registerPDO(TPDO_1a1d);
      sm(3).registerPDO(TPDO_1a1e);
      sm(3).registerPDO(TPDO_1a22);

   }

   public void setTargetPosition(int position)
   {
      rpdo_1606.targetPosition.set(position);
   }

   public void setTargetVelocity(int velocity)
   {
      rpdo_1606.velocityOffset.set(velocity);
   }

   public void setTargetTorque(double torque)
   {
      rpdo_1606.torqueOffset.set((short) (((long) (torque * 1000000.0)) / maxDriveTorque));

   }

   public void setModeOfOperation(ElmoModeOfOperation mode)
   {

      rpdo_160B.modeOfOperation.set(mode.getMode());

   }

   public boolean isElmoOperational()
   {
      return this.isOperational();
   }

   public void setDigitalOutput(int output, boolean val)
   {
      if (output < 0 || output > 5)
      {
         throw new RuntimeException("Invalid digital output " + output);
      }

      int shift = 16 + output;
      if (val)
      {
         //Bit-wise OR to set bit 18 to 1
         digitalOutputBitString = digitalOutputBitString | (1 << shift);
      }
      else
      {
         //Bit-wise AND to set bit 18 to 0 (NOTE: 4294967295 is the max value for unsigned 32-bit int)
         digitalOutputBitString = digitalOutputBitString & ((1 << shift) ^ 4294967295l);
      }

      rpdo_1606.digitalOutputs.set(digitalOutputBitString);

   }

   public int getActualPosition()
   {
      return TPDO_1a03.positionActualValue.get();
   }

   public long getDigitalInputs()
   {
      return TPDO_1a03.digitalInputs.get();
   }

   public int getActualVelocity()
   {
      return TPDO_1a03.velocityActualValue.get();
   }

   public int getStatusVar()
   {
      return TPDO_1a03.statusWord.get();
   }

   public double getActualTorque()
   {
      return (((double) TPDO_1a13.torqueActualValue.get() * (double) maxDriveTorque) / 1000000.0);
   }

   public int getActualAuxiliaryPosition()
   {
      return TPDO_1a1e.auxiliaryPositionActualValue.get();
   }

   protected Unsigned16 getStatusWordPDOEntry()
   {
      return TPDO_1a03.statusWord;
   }

   protected Unsigned16 getControlWordPDOEntry()
   {
      return rpdo_1606.controlWord;
   }

   public int getAnalogInputVoltageInMilliVolts()
   {
      return TPDO_1a1d.analogInput.get();
   }

   public long getTorqueScale()
   {
      return maxDriveTorque;
   }

   public double getCommandedCurrent()
   {
      return (((double) TPDO_1a12.torqueDemand.get() * (double) maxDriveTorque) / 1000000.0);
   }

   public double getActualCurrent()
   {
      return getActualTorque();
   }

   public int getMotorEncoderPosition()
   {
      return getActualPosition();
   }

   public int getLoadEncoderPosition()
   {
      return getActualAuxiliaryPosition();
   }

   public int getMotorEncoderVelocity()
   {
      return getActualVelocity();
   }

   public long getElmoStatusRegister()
   {
      return TPDO_1a22.elmoStatusRegister.get();
   }
   
   public long getDCLinkVoltageMilliVolts()
   {
      return TPDO_1a18.dcLinkVoltage.get();
   }

   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      writeSDO(0x60c2, 0x1, 2);

      maxDriveTorque = readSDOUnsignedInt(0x6076, 0x0);
      System.out.println(toString() + " max drive torque is " + maxDriveTorque);
      
      if(dcEnabled)
      {
         configureDCSync0(true, cycleTimeInNs, 0);
      }
      else
      {
         configureDCSync0(false, 0, 0);
      }
   }
}
