package us.ihmc.etherCAT.slaves.ihmc;

import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

public class IHMCEtherCATIMU extends Slave
{
   
   private final static int vendor = 0x00000603;
   private final static int productCode = 0x00000030;
   
   private class Outputs extends RxPDO
   {
      public Outputs()
      {
         super(0x1600);
      }
      
      private final Bool[] out = array(new Bool[8]);
      private final Unsigned8 gap = new Unsigned8();
   }
   
   private final Outputs outputs = new Outputs();
   
   private class IMUData extends TxPDO
   {
      public IMUData()
      {
         super(0x1a00);
      }
      
      private final Unsigned16 packetCounter = new Unsigned16();
      private final Unsigned32 sampleTime = new Unsigned32();
      private final Unsigned32 status = new Unsigned32();
      private final Float32 qx = new Float32();
      private final Float32 qy = new Float32();
      private final Float32 qz = new Float32();
      private final Float32 qs = new Float32();
      private final Float32 xdd = new Float32();
      private final Float32 ydd = new Float32();
      private final Float32 zdd = new Float32();
      private final Float32 wx = new Float32();
      private final Float32 wy = new Float32();
      private final Float32 wz = new Float32();
      
   }
   private final IMUData imuData = new IMUData();
   
   private class Inputs extends TxPDO
   {
      public Inputs()
      {
         super(0x1a10);
      }
      
      private final Bool[] in = array(new Bool[8]);
      private final Unsigned8 gap = new Unsigned8();
      private final Float32 busVoltage = new Float32();
   }
   
   private final Inputs inputs = new Inputs();

   public IHMCEtherCATIMU(int aliasAddress, int position)
   {
      super(vendor, productCode, aliasAddress, position);
      
      registerSyncManager(new SyncManager(2, false));
      registerSyncManager(new SyncManager(3, false));
      
      sm(2).registerPDO(outputs);
      sm(3).registerPDO(imuData);
      sm(3).registerPDO(inputs);
   }
   



   public boolean getDigitalInput(int input)
   {
      return inputs.in[input].get();
   }


   public float getBusVoltage()
   {
      return inputs.busVoltage.get();
   }


   public long getIMU0StatusWord()
   {
      return imuData.status.get();
   }


   public int getIMU0PacketCounter()
   {
      return imuData.packetCounter.get();
   }


   public long getIMU0SampleTime()
   {
      return imuData.sampleTime.get();
   }


   public double getIMU0qx()
   {
      return imuData.qx.get();
   }


   public double getIMU0qy()
   {
      return imuData.qy.get();
   }


   public double getIMU0qz()
   {
      return imuData.qz.get();
   }


   public double getIMU0qs()
   {
      return imuData.qs.get();
   }


   public double getIMU0xdd()
   {
      return imuData.xdd.get();
   }


   public double getIMU0ydd()
   {
      return imuData.ydd.get();
   }


   public double getIMU0zdd()
   {
      return imuData.zdd.get();
   }


   public double getIMU0wx()
   {
      return imuData.wx.get();
   }


   public double getIMU0wy()
   {
      return imuData.wy.get();
   }


   public double getIMU0wz()
   {
      return imuData.wz.get();
   }
   
   public void setOutput(int output, boolean value)
   {
      outputs.out[output].set(value);
   }
   
   
}
