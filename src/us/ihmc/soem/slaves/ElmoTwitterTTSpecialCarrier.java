package us.ihmc.soem.slaves;

import java.io.IOException;
import java.util.Random;

import javolution.io.Struct.Unsigned16;
import us.ihmc.soem.wrapper.Master;
import us.ihmc.soem.wrapper.RxPDO;
import us.ihmc.soem.wrapper.SyncManager;
import us.ihmc.soem.wrapper.TxPDO;

public class ElmoTwitterTTSpecialCarrier extends DSP402Slave
{
   static final int vendorID = 0x0000009a;
   static final int productCode = 0x00030924;
   static final int assignActivate = 0x0300;

   // Variable to determine SDO timing, randomized to distribute SDO requests over control cycle
   private int readTick = new Random().nextInt(1000);
   private final int readSDOEveryNTicks;

   //Status Register Event Locations
   private final long UNDER_VOLTAGE = 3;
   private final long OVER_VOLTAGE = 5;
   private final long STO_DISABLED = 7;
   private final long CURRENT_SHORT = 11;
   private final long OVER_TEMPERATURE = 13;
   private final long MOTOR_ENABLED = 16;
   private final long MOTOR_FAULT = 64;
   private final long CURRENT_LIMITED = 8192;


   private class RPDO_1606 extends RxPDO
   {
      protected RPDO_1606()
      {
         super(0x1606);
      }

      Signed32 targetPosition = new Signed32();
      Unsigned32 digitalOutputs = new Unsigned32();
      Signed32 targetVelocity = new Signed32();
      Signed32 velocityOffset = new Signed32();
      Signed16 torqueOffset = new Signed16();
      Unsigned16 controlWord = new Unsigned16();
   }

   private final RPDO_1606 rpdo_1606 = new RPDO_1606();

   private class RPDO_160B extends RxPDO
   {
      protected RPDO_160B()
      {
         super(0x160B);
      }

      Signed8 modeOfOperation = new Signed8();
      Unsigned8 dummy = new Unsigned8();
   }

   private final RPDO_160B rpdo_160B = new RPDO_160B();

   private class TPDO_1a03 extends TxPDO
   {
      protected TPDO_1a03()
      {
         super(0x1a03);
      }

      Signed32 positionActualValue = new Signed32();
      Unsigned32 digitalInputs = new Unsigned32();
      Signed32 velocityActualValue = new Signed32();
      Unsigned16 statusWord = new Unsigned16();

   }

   private final TPDO_1a03 tpdo_1a03 = new TPDO_1a03();

   private class TPDO_1a13 extends TxPDO
   {
      protected TPDO_1a13()
      {
         super(0x1a13);
      }

      Signed16 torqueActaulValue = new Signed16();
   }

   private final TPDO_1a13 tpdo_1a13 = new TPDO_1a13();

   private class TPDO_1a19 extends TxPDO
   {
      protected TPDO_1a19()
      {
         super(0x1a19);
      }

      Signed32 positionFollowingError = new Signed32();
   }

   private final TPDO_1a19 tpdo_1a19 = new TPDO_1a19();

   private class TPDO_1a1e extends TxPDO
   {
      protected TPDO_1a1e()
      {
         super(0x1a1e);
      }

      Signed32 auxiliaryPositionActualValue = new Signed32();
   }

   private final TPDO_1a1e tpdo_1a1e = new TPDO_1a1e();

   private class TPDO_1a22 extends TxPDO
   {
      protected TPDO_1a22()
      {
         super(0x1a22);
      }

      Unsigned32 elmoStatusRegister = new Unsigned32();
   }

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

   public ElmoTwitterTTSpecialCarrier(Master master, int alias, boolean enableDC, long cyclicPeriodNS)
   {
      this(master, alias, 0, enableDC, cyclicPeriodNS);
   }

   /**
    * 
    * @param domain
    * @param alias
    * @param ringPosition
    * @param enableDC enable DC or not, twitter will not reliable enable without DC
    * @param cyclicPeriodNS
    * @throws IOException
    */
   public ElmoTwitterTTSpecialCarrier(Master master, int alias, int ringPosition, boolean enableDC, long cyclicPeriodNS)
   {
      super(master, alias, ringPosition);

      // Read SDO variables every second
      readSDOEveryNTicks = (int) (1000000000 / cyclicPeriodNS);

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
      

      //      configureWatchdog((int) (cyclicPeriodNS / 40), 100);
      //      
      //      if(enableDC)
      //      {
      //         //Enable Distributed Clock. Twitter does not send data without DC.
      //         configDC(assignActivate, cyclicPeriodNS, 0, 0, 0);      
      //      }
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
      return tpdo_1a13.torqueActaulValue.get();
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

   private boolean getStatusValue(long maskValue)
   {
      //Bitwise AND to check bit-string for status events 
      return (getElmoStatusRegister() & maskValue) == maskValue;
   }

   public int getPositionFollowingError()
   {
      return tpdo_1a19.positionFollowingError.get();
   }

   public boolean isFaulted()
   {
      return isUnderVoltage() || isOverVoltage() || isSTODisabled() || isCurrentShorted() || isOverTemperature() || isMotorFaulted();
   }

   //Status Register Events
   public boolean isUnderVoltage()
   {
      return getStatusValue(UNDER_VOLTAGE);
   }

   public boolean isOverVoltage()
   {
      return getStatusValue(OVER_VOLTAGE);
   }

   public boolean isSTODisabled()
   {
      return getStatusValue(STO_DISABLED);
   }

   public boolean isCurrentShorted()
   {
      return getStatusValue(CURRENT_SHORT);
   }

   public boolean isOverTemperature()
   {
      return getStatusValue(OVER_TEMPERATURE);
   }

   public boolean isMotorEnabled()
   {
      return getStatusValue(MOTOR_ENABLED);
   }

   public boolean isMotorFaulted()
   {
      return getStatusValue(MOTOR_FAULT);
   }

   public boolean isCurrentLimited()
   {
      return getStatusValue(CURRENT_LIMITED);
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
}
