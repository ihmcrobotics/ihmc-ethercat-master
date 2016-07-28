package us.ihmc.soem.slaves.copley;

import java.io.IOException;

import us.ihmc.soem.javalution.Struct.Unsigned16;
import us.ihmc.soem.javalution.Union;
import us.ihmc.soem.slaves.DSP402Slave;
import us.ihmc.soem.wrapper.RxPDO;
import us.ihmc.soem.wrapper.SyncManager;
import us.ihmc.soem.wrapper.TxPDO;

public class CopleyAEM extends DSP402Slave
{
   private final long ENCODER_ERROR = (long) Math.pow(2, 5);
   private final long CURRENT_LIMITED = (long) Math.pow(2, 7);
   private final long UNDER_VOLTAGE = (long) Math.pow(2, 3);
   private final long OVER_VOLTAGE = (long) Math.pow(2, 2);
   private final long OUTPUT_VOLTAGE_LIMITED = (long) Math.pow(2, 8);



   static final int vendorID = 0x000000ab;
   static final int productCode = 0x00001030;
   
   public enum CaptureIndexPositionMode
   {
      FALLING_EDGE,
      RISING_EDGE,
      DISABLED
   }


   public class RPDO1 extends RxPDO
   {
      public RPDO1()
      {
         super(0x1702);
      }
      
      Unsigned16 controlWord = new Unsigned16();
      Signed16 targetTorque = new Signed16();
      Unsigned16 positionCaptureControl = new Unsigned16();
      Unsigned16 digitalOutputState = new Unsigned16();
      
   }
   private final RPDO1 rpdo1 = new RPDO1();
   
   public class TPDO1 extends TxPDO
   {
      public TPDO1()
      {
         super(0x1A03);
      }
      
      Unsigned32 amplifierEventStatusRegister = new Unsigned32();
      Signed32 actualLoadEncoderVelocity = new Signed32();
      Signed16 commandedCurrent = new Signed16();
      Signed16 actualCurrent = new Signed16();
      Signed16 positionCaptureStatusRegister = new Signed16();
      Signed32 capturedIndexPosition = new Signed32();
      
      RxPosition position = inner(new RxPosition());
      
      public class RxPosition extends Union
      {
         Signed32 actualLoadEncoderPosition = new Signed32();
         Signed32 actualMotorEncoderPosition = new Signed32();
      }
      
      
   }
   private final TPDO1 tpdo1 = new TPDO1();
   
   public class TPDO2 extends TxPDO
   {
      public TPDO2()
      {
         super(0x1B00);
      }
      
      Unsigned16 statusWord = new Unsigned16();
      TxPosition position = inner(new TxPosition());
      Signed32 positionLoopError = new Signed32();
      Signed32 actualMotorEncoderVelocity = new Signed32();
      Signed16 torqueActualValue = new Signed16();
      
      public class TxPosition extends Union
      {
         Signed32 actualLoadEncoderPosition = new Signed32();
         Signed32 actualMotorEncoderPosition = new Signed32();

      }
      
   }
   private final TPDO2 tpdo2 = new TPDO2();
   
//   private final TxPDOEntry outputConfigureTransmit0;
//   private final TxPDOEntry outputConfigureTransmit1;
//   private final TxPDOEntry outputConfigureTransmit2;
//   private final TxPDOEntry outputConfigureTransmit3;
//   private final TxPDOEntry outputConfigureTransmit4;
//   private final TxPDOEntry outputConfigureTransmit5;
//   private final TxPDOEntry outputConfigureTransmit6;
//   private final TxPDOEntry digitalOutputStateTransmit;

   private int positionCaptureControlValue = 0;
   private int digitalOutputStateValue = 0;
   
//   private final TxPDOEntry inputPins;
   
   public CopleyAEM(int alias, boolean loadEncoderIsPassive) throws IOException
   {
      this(alias, 0, loadEncoderIsPassive);
   }

   public CopleyAEM(int alias, int ringPosition, boolean loadEncoderIsPassive) throws IOException
   {
      super(vendorID, productCode, alias, ringPosition);
      
      for(int i = 0; i < 4; i++)
      {
         registerSyncManager(new SyncManager(i, true));
      }
      sm(2).registerPDO(rpdo1);
      sm(3).registerPDO(tpdo1);
      sm(3).registerPDO(tpdo2);

//      addSDOConfiguration(0x2193, 0x1, DataType.U16, 2);
//      addSDOConfiguration(0x2193, 0x2, DataType.U16, 2);
//      addSDOConfiguration(0x2193, 0x3, DataType.U16, 2);
//      addSDOConfiguration(0x2193, 0x4, DataType.U16, 2);
//      addSDOConfiguration(0x2193, 0x5, DataType.U16, 2);
//      addSDOConfiguration(0x2193, 0x6, DataType.U16, 2);
      
      
   }
   
   public boolean isEncoderError()
   {
//      return (amplifierEventStatusRegister.getValueAsLong() & 0x20) == 0x20;
      return getBitValue(ENCODER_ERROR); 
   }
   
   public boolean isCurrentLimited()
   {
      return getBitValue(CURRENT_LIMITED);
   }
   
   public boolean isUnderVoltage()
   {
      return getBitValue(UNDER_VOLTAGE);
   }

   public boolean isOverVoltage()
   {
      return getBitValue(OVER_VOLTAGE);
   }

   public boolean isOutputVoltageLimited()
   {
      return getBitValue(OUTPUT_VOLTAGE_LIMITED);
   }

   private boolean getBitValue(long maskValue)
   {
      return (tpdo1.amplifierEventStatusRegister.get() & maskValue) == maskValue;  
   }
   
   public void setTorque(int desiredTorque) 
   {
      rpdo1.targetTorque.set((short)desiredTorque);
   }

   public int getMotorEncoderPosition()
   {
      return tpdo1.position.actualMotorEncoderPosition.get();
   }
   
   public double getCommandedCurrent()
   {
      return ((double) tpdo1.commandedCurrent.get()) * 0.01;
   }
   public double getActualCurrent()
   {
      return ((double) tpdo1.actualCurrent.get()) * 0.01;
   }
   public int getLoadEncoderPosition()
   {
      return tpdo1.position.actualLoadEncoderPosition.get();
   }

   public int getMotorEncoderVelocity()
   {
      return tpdo2.actualMotorEncoderVelocity.get();
   }
   
   public int getLoadEncoderVelocity()
   {
      return tpdo1.actualLoadEncoderVelocity.get();
   }
   
   public int getActualTorque()
   {
      return tpdo2.torqueActualValue.get();
   }
   
   public void setCaptureIndexPositonMode(CaptureIndexPositionMode captureIndexPositionMode)
   {
      boolean captureOnFallingEdge = false;
      boolean captureOnRisingEdge = false;
      
      switch(captureIndexPositionMode)
      {
      case RISING_EDGE:
         captureOnRisingEdge = true;
         break;
      case FALLING_EDGE:
         captureOnFallingEdge = true;
         break;
      case DISABLED:
         ;;
         break;
      }
      
      setPositionCaptureControlRegister(0, captureOnFallingEdge);
      setPositionCaptureControlRegister(1, captureOnRisingEdge);
      
      updatePositionCaptureControlRegister();
   }
   
   public void setClearOnIndex(boolean clearOnIndex)
   {
      setPositionCaptureControlRegister(12, clearOnIndex);
      updatePositionCaptureControlRegister();
   }
   
   
   public boolean hasSeenIndexPulse()
   {
      return (tpdo1.positionCaptureStatusRegister.get() & (1 << 0)) == 1; 
   }
   
      
   public void setDigitalOutput(int outputPort, boolean enable)
   {
             
      if(outputPort < 0 || outputPort > 15)
      {
         throw new RuntimeException("Invalid output port");
      }
      
      if(enable)
      {
         digitalOutputStateValue |= (1 << outputPort);
      }
      else
      {
         digitalOutputStateValue &= ~(1 << outputPort);
      }
      
        rpdo1.digitalOutputState.set(digitalOutputStateValue);
         
//            System.out.println("Out = " + Integer.toBinaryString(digitalOutputStateValue) + ", outputConfigureTransmit = " + Integer.toBinaryString((int) outputConfigureTransmit.getValueAsLong()) + ", digitalOutputStateTransmit = " + Integer.toBinaryString((int) digitalOutputStateTransmit.getValueAsLong()));
//            System.out.println("digitalOutR = " + Integer.toBinaryString(digitalOutputStateValue) 
//                              + ", digitalOutT = " + Integer.toBinaryString((int) digitalOutputStateTransmit.getValueAsLong())
//                              + ", out0 = " + Integer.toBinaryString((int) outputConfigureTransmit0.getValueAsLong())
//                              + ", out1 = " + Integer.toBinaryString((int) outputConfigureTransmit1.getValueAsLong())
//                              + ", out2 = " + Integer.toBinaryString((int) outputConfigureTransmit2.getValueAsLong())
//                              + ", out3 = " + Integer.toBinaryString((int) outputConfigureTransmit3.getValueAsLong())
//                              + ", out4 = " + Integer.toBinaryString((int) outputConfigureTransmit4.getValueAsLong())
//                              + ", out5 = " + Integer.toBinaryString((int) outputConfigureTransmit5.getValueAsLong())
//                              + ", out6 = " + Integer.toBinaryString((int) outputConfigureTransmit6.getValueAsLong())
//                              + ", inputPins = " + Integer.toBinaryString((int) inputPins.getValueAsLong())
//                              );
//            System.out.println("inputPins = " + Integer.toBinaryString((int) inputPins.getValueAsLong()));

   }
   
   //Added this
   public int getDigitalOutput()
   {
      return digitalOutputStateValue;
   }
   
   private void setPositionCaptureControlRegister(int bitIndex, boolean value)
   {
      if(value)
      {
         positionCaptureControlValue |= (1 << bitIndex);
      }
      else
      {
         positionCaptureControlValue &= ~(1 << bitIndex);
      }
   }
   
   private void updatePositionCaptureControlRegister()
   {
         rpdo1.positionCaptureControl.set(positionCaptureControlValue);
   }
   

   protected Unsigned16 getStatusWordPDOEntry()
   {
      return tpdo2.statusWord;
   }

   protected Unsigned16 getControlWordPDOEntry()
   {
      return rpdo1.controlWord;
   }

   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      writeSDO(0x6060, 0x0, (byte) 10);
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
