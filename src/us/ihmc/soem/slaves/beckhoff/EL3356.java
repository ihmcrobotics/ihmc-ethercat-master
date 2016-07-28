package us.ihmc.soem.slaves.beckhoff;

import java.io.IOException;

import us.ihmc.soem.wrapper.RxPDO;
import us.ihmc.soem.wrapper.Slave;
import us.ihmc.soem.wrapper.SyncManager;
import us.ihmc.soem.wrapper.TxPDO;

public class EL3356 extends Slave
{
   public enum FilterSetting
   {
      DEACTIVATED,
      FIR50,   // 50Hz FIR notch filter
      FIR60,   // 60Hz FIR notch filter
      IIR1,    // 2000 Hz low pass
      IIR2,    // 500 Hz low pass
      IIR3,    // 125 Hz low pass
      IIR4,    // 30 Hz low pass
      IIR5,    // 8 Hz low pass
      IIR6,    // 2 Hz low pass
      IIR7,    // 0.5 Hz low pass
      IIR8,    // 0.1 Hz low pass
      DYNAMICIIR;  // Dynamic change between IIR1 and IIR8
      
      public int getIntegerValue()
      {
         switch(this)
         {
         
         case FIR50:
            return 0;
         case FIR60:
            return 1;
         case IIR1:
            return 2;
         case IIR2:
            return 3;
         case IIR3:
            return 4;
         case IIR4:
            return 5;
         case IIR5:
            return 6;
         case IIR6:
            return 7;
         case IIR7:
            return 8;
         case IIR8:
            return 9;
         case DYNAMICIIR:
            return 10;
         case DEACTIVATED:
         default:
            throw new RuntimeException("No integer value for deactived filter");               
            
         }
      }
   }
   
   static final int vendorID = 0x00000002;
   static final int productCode = 0x0d1c3052;
   
   public class RMBControl extends RxPDO
   {
      public RMBControl()
      {
         super(0x1600);
      }
      
      Bool startCalibration = new Bool();
      Bool disableCalibration = new Bool();
      Bool inputFreeze = new Bool();
      Bool sampleMode = new Bool();
      Bool tara = new Bool();
      Unsigned8 gap0 = new Unsigned8();
      Bit3 gap1 = new Bit3();
   }
   private final RMBControl rmbControl = new RMBControl();
   
   public class RMBStatus extends TxPDO
   {
      public RMBStatus()
      {
         super(0x1a00);
      }
      
      
      Bool gap0 = new Bool();
      Bool overRange = new Bool();
      Bool gap2 = new Bool();
      Bool dataInvalid = new Bool();
      Bit2 gap3 = new Bit2();
      Bool error = new Bool();
      Bool calibrationInProgress = new Bool();
      Bool steadystate = new Bool();
      Bit4 gap4 = new Bit4();
      Bool syncError = new Bool();
      Bool gap5 = new Bool();
      Bool txPDOToggle = new Bool();
      
      
   }
   private final RMBStatus rmbStatus =  new RMBStatus();
   
   public class RMBValue extends TxPDO
   {
      public RMBValue()
      {
         super(0x1A02);
      }
      
      Float32 value = new Float32();
   }
   
   private final RMBValue rmbValue = new RMBValue();
   
   private final double nominalSensitivity, zeroBalance, nominalLoad;
   private final boolean enableAverager;
   private final FilterSetting filterSetting;
   
   /**
    * 
    *  @see EL3356
    */
   public EL3356(int alias, double nominalSensitivity, double zeroBalance, double nominalLoad, boolean enableAverager, FilterSetting filterSetting) throws IOException
   {
      this(alias, 0, nominalSensitivity, zeroBalance, nominalLoad, enableAverager, filterSetting);

   }
   
   /**
    * 
    * Create a new load cell object. 
    * 
    *    Do not forget to enable high speed mode if desired
    *    Disable calibration for better real time behaviour.
    * 
    * @param domain
    * @param alias   Alias of slave
    * @param position   Bus position
    * @param nominalSensitivity  Sensitivity of force sensor [mV/V]
    * @param zeroBalance   Zero balance [mV/V]
    * @param nominalLoad   Nominal load of sensor [N, kg, ...] 
    * @param enableAverager   Enable 4 data point averager which runs at 10kHz
    * @param filterSetting    Type of filter
    * @throws IOException
    */
   public EL3356(int alias, int position, double nominalSensitivity, double zeroBalance, double nominalLoad, boolean enableAverager, FilterSetting filterSetting) throws IOException
   {
      super(vendorID, productCode, alias, position);
      
      registerSyncManager(new SyncManager(3, true));
      registerSyncManager(new SyncManager(2, true));
      sm(3).registerPDO(rmbStatus);
      sm(3).registerPDO(rmbValue);
      
      sm(2).registerPDO(rmbControl);
      
      
      this.nominalSensitivity = nominalSensitivity;
      this.zeroBalance = zeroBalance;
      this.nominalLoad = nominalLoad;
      this.enableAverager = enableAverager;
      this.filterSetting = filterSetting;
      

      

   }
   
   public double value()
   {
      return rmbValue.value.get();
   }
   
   public void enableHighSpeedMode()
   {
      rmbControl.sampleMode.set(true);
   }
   
   public void disableHighSpeedMode()
   {
      rmbControl.sampleMode.set(false);
   }
   
   public void enableCalibration()
   {
      rmbControl.disableCalibration.set(false);
   }
   
   public void disableCalibration()
   {
      rmbControl.disableCalibration.set(true);
   }
   
   public void startCalibration()
   {
      rmbControl.startCalibration.set(true);
   }
   
   public void stopCalibration()
   {
      rmbControl.startCalibration.set(false);
   }
   
   public void freezeInput()
   {
      rmbControl.inputFreeze.set(true);
   }
   
   public void resumeInput()
   {
      rmbControl.inputFreeze.set(false);
   }
   
   public boolean isOverRange()
   {
      return rmbStatus.overRange.value();
   }
   
   public boolean isDataInvalid()
   {
      return rmbStatus.dataInvalid.value();
   }
   
   public boolean isError()
   {
      return rmbStatus.error.value();
   }
   
   public boolean isCalibrationInProgress()
   {
      return rmbStatus.calibrationInProgress.value();
   }
   
   public boolean isSteadyState()
   {
      return rmbStatus.steadystate.value();
   }
   
   public boolean isSyncError()
   {
      return rmbStatus.syncError.value();
   }

   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      writeSDO(0x8000, 0x23, (float)nominalSensitivity);
      writeSDO(0x8000, 0x24, (float)nominalLoad);
      writeSDO(0x8000, 0x25, (float)zeroBalance);
      
      
      // Sensible default gains and gravity
      writeSDO(0x8000, 0x21, 1.0f);
      writeSDO(0x8000, 0x26, 9.806650f);
      writeSDO(0x8000, 0x27, 1.0f);
      
      
      if(filterSetting == FilterSetting.DEACTIVATED)
      {
         writeSDO(0x8000, 0x01, (byte)0);
         writeSDO(0x8000, 0x02, (byte) 0);
      }
      else
      {
         writeSDO(0x8000, 0x01, (byte) 1);
         writeSDO(0x8000, 0x02, (byte) 1);
         writeSDO(0x8000, 0x11, (short)filterSetting.getIntegerValue());
         writeSDO(0x8000, 0x12, (short) filterSetting.getIntegerValue());
      }
      
      
      writeSDO(0x8000, 0x03, (byte)(enableAverager?(byte)1:(byte)0));
      writeSDO(0x8000, 0x04, (byte)(enableAverager?(byte)1:(byte)0));
      
      super.configure(dcEnabled, cycleTimeInNs);
   }
   
}
