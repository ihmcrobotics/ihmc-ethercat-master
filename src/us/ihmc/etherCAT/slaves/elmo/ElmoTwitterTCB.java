package us.ihmc.etherCAT.slaves.elmo;

import java.io.IOException;

import us.ihmc.etherCAT.javalution.Struct.Unsigned16;
import us.ihmc.etherCAT.master.SyncManager;

public class ElmoTwitterTCB extends ElmoTwitter
{
   private final RPDO_1606 rpdo_1606 = new RPDO_1606();
   private final RPDO_160B rpdo_160B = new RPDO_160B();

   private final TPDO_1a03 tpdo_1a03 = new TPDO_1a03();
   private final TPDO_1a13 tpdo_1a13 = new TPDO_1a13();
   private final TPDO_1a19 tpdo_1a19 = new TPDO_1a19();
   private final TPDO_1a1e tpdo_1a1e = new TPDO_1a1e();
   private final TPDO_1a22 tpdo_1a22 = new TPDO_1a22();

   //SDO data
   //   private final SDO analogInput2;
   //   private final SDO twitterTemperature;
   //   private final SDO errorCode;
   private int analogInput2Value;
   private int twitterTemperatureValue;
   private int errorCodeValue;

   //Digital Outputs
   private long digitalOutputBitString = 0l;

   private boolean hasBeenEnabled = false;

   /**
    * 
    * @param domain
    * @param alias
    * @param ringPosition
    * @param enableDC enable DC or not, twitter will not reliable enable without DC
    * @param cyclicPeriodNS
    * @throws IOException
    */
   public ElmoTwitterTCB(int alias, int ringPosition)
   {
      super(alias, ringPosition);

      //Sync Manager, only SM2 and SM3 are used for PDOs.  SM0 and SM1 are used for the mailbox mechanism (See: EtherCAT_Application_Manual.pdf)
      for(int i = 0; i < 4; i++)
      {
         registerSyncManager(new SyncManager(i, true));
      }

      sm(2).registerPDO(rpdo_1606);
      sm(2).registerPDO(rpdo_160B);
      sm(3).registerPDO(tpdo_1a03);
      sm(3).registerPDO(tpdo_1a13);
      sm(3).registerPDO(tpdo_1a19);
      sm(3).registerPDO(tpdo_1a1e);
      sm(3).registerPDO(tpdo_1a22);
   }

   public void setTargetPosition(int position)
   {

      rpdo_1606.targetPosition.set(position);

   }

   public void setTargetVelocity(int velocity)
   {
      rpdo_1606.targetVelocity.set(velocity);

   }

   public void setModeOfOperation(ElmoModeOfOperation mode)
   {
      rpdo_160B.modeOfOperation.set(mode.getMode());

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

   public int getPositionActualValue()
   {
      return tpdo_1a03.positionActualValue.get();
   }

   public int getDigitalInputs()
   {
      return (int) tpdo_1a03.digitalInputs.get();
   }

   public int getVelocityActualValue()
   {
      return tpdo_1a03.velocityActualValue.get();
   }

   public long getStatusVar()
   {
      return tpdo_1a22.elmoStatusRegister.get();
   }

   public int getActualTorque()
   {
      return tpdo_1a13.torqueActualValue.get();
   }

   public int getActualAuxiliaryPosition()
   {
      return tpdo_1a1e.auxiliaryPositionActualValue.get();
   }

   protected Unsigned16 getStatusWordPDOEntry()
   {
      return tpdo_1a03.statusWord;
   }

   protected Unsigned16 getControlWordPDOEntry()
   {
      return rpdo_1606.controlWord;
   }

   @Override
   public void doStateControl()
   {
      if (!hasBeenEnabled)
      {
         hasBeenEnabled = getStatus() == StatusWord.OPERATIONENABLE;
      }


      super.doStateControl();
   }

   public double getActualCurrent()
   {
      return getActualTorque();
   }

   public int getMotorEncoderPosition()
   {
      return getPositionActualValue();
   }

   public int getLoadEncoderPosition()
   {
      return getActualAuxiliaryPosition();
   }

   public int getMotorEncoderVelocity()
   {
      return getVelocityActualValue();
   }

   public long getElmoStatusRegister()
   {
      return tpdo_1a22.elmoStatusRegister.get();
   }

   public int getElmoErrorCode()
   {
      return errorCodeValue;
   }


   public int getPositionFollowingError()
   {
      return tpdo_1a19.positionFollowingError.get();
   }

   public void setVelocityOffset(int integerValue)
   {
      rpdo_1606.velocityOffset.set(integerValue);
   }

   public void setTorqueOffset(int integerValue)
   {
      rpdo_1606.torqueOffset.set((short)integerValue);
   }

   public int getSlowTwitterTemperature()
   {
      return twitterTemperatureValue;
   }

   public int getSlowAnalogInput2()
   {
      return analogInput2Value;
   }


   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
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
