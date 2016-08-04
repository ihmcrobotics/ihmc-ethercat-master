package us.ihmc.etherCAT.slaves;

import java.io.IOException;

import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

public class IHMCBlueBox extends Slave
{
   static final int vendorID = 0x00000500;
   static final int productCode = 0x00904001;
   
   
   private class ReceivePDO extends RxPDO
   {
      public ReceivePDO()
      {
         super(0x1600);
      }
   }
   private final ReceivePDO receivePDO = new ReceivePDO();
   
   public class TransmitPDO extends TxPDO
   
   {
      public TransmitPDO()
      {
         super(0x1a00);
      }
      
      Unsigned8 ADStatus = new Unsigned8();
      Unsigned8 counter =  new Unsigned8();
      Unsigned8 digitalInputs =  new Unsigned8();
      Signed16[] analogInputs = array(new Signed16[8]);
      
      Unsigned32 IMU0StatusWord =  new Unsigned32();
      Unsigned16 IMU0PacketCounter = new Unsigned16();
      Unsigned32 IMU0SampleTimeFine = new Unsigned32();
      Float32 IMU0qx = new Float32();
      Float32 IMU0qy = new Float32();
      Float32 IMU0qz = new Float32();
      Float32 IMU0qs = new Float32();
      
      Float32 IMU0xdd = new Float32();
      Float32 IMU0ydd = new Float32();
      Float32 IMU0zdd = new Float32();

      Float32 IMU0wx = new Float32();
      Float32 IMU0wy = new Float32();
      Float32 IMU0wz = new Float32();
   }
   
   private final TransmitPDO transmitPDO = new TransmitPDO();
   

   
   
   public IHMCBlueBox(int alias, int ringPosition)
   {
      super(vendorID, productCode, alias, ringPosition);
      
      for (int i = 2; i < 4; i++)
      {
         registerSyncManager(new SyncManager(i, true));
      }

      
      sm(2).registerPDO(receivePDO);
      sm(3).registerPDO(transmitPDO);
      
   }


   public int getADStatus()
   {
      return transmitPDO.ADStatus.get();
   }


   public int getCounter()
   {
      return transmitPDO.counter.get();
   }


   public int getDigitalInputs()
   {
      return transmitPDO.digitalInputs.get();
   }


   public int getAnalogInputs(int input)
   {
      return transmitPDO.analogInputs[input].get();
   }


   public long getIMU0StatusWord()
   {
      return transmitPDO.IMU0StatusWord.get();
   }


   public int getIMU0PacketCounter()
   {
      return transmitPDO.IMU0PacketCounter.get();
   }


   public long getIMU0SampleTimeFine()
   {
      return transmitPDO.IMU0SampleTimeFine.get();
   }


   public double getIMU0qx()
   {
      return transmitPDO.IMU0qx.get();
   }


   public double getIMU0qy()
   {
      return transmitPDO.IMU0qy.get();
   }


   public double getIMU0qz()
   {
      return transmitPDO.IMU0qz.get();
   }


   public double getIMU0qs()
   {
      return transmitPDO.IMU0qs.get();
   }


   public double getIMU0xdd()
   {
      return transmitPDO.IMU0xdd.get();
   }


   public double getIMU0ydd()
   {
      return transmitPDO.IMU0ydd.get();
   }


   public double getIMU0zdd()
   {
      return transmitPDO.IMU0zdd.get();
   }


   public double getIMU0wx()
   {
      return transmitPDO.IMU0wx.get();
   }


   public double getIMU0wy()
   {
      return transmitPDO.IMU0wy.get();
   }


   public double getIMU0wz()
   {
      return transmitPDO.IMU0wz.get();
   }
   

}
