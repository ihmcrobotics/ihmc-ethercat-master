package us.ihmc.soem.slaves.beckhoff;

import us.ihmc.soem.wrapper.RxPDO;
import us.ihmc.soem.wrapper.Slave;
import us.ihmc.soem.wrapper.SyncManager;

public class EL4134 extends Slave
{

   static final int vendorID = 0x00000002;
   static final int productCode = 0x10263052;
   
   class Output extends RxPDO
   {

      protected Output(int address)
      {
         super(address);
      }
      
      Signed16 value = new Signed16();

   };

   private final Output out1 = new Output(0x1600);
   private final Output out2 = new Output(0x1601);
   private final Output out3 = new Output(0x1602);
   private final Output out4 = new Output(0x1603);

   public EL4134(int aliasAddress, int configAddress)
   {
      super(vendorID, productCode, aliasAddress, configAddress);

      registerSyncManager(new SyncManager(2, false));

      sm(2).registerPDO(out1);
      sm(2).registerPDO(out2);
      sm(2).registerPDO(out3);
      sm(2).registerPDO(out4);

   }

   public void setOut1(int val)
   {
      out1.value.set((short) val);
   }

   public void setOut2(int val)
   {
      out2.value.set((short) val);
   }

   public void setOut3(int val)
   {
      out3.value.set((short) val);
   }

   public void setOut4(int val)
   {
      out4.value.set((short) val);
   }

   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      if(dcEnabled)
      {
         configureDCSync01(true, cycleTimeInNs, 100000, 0);
      }
      else
      {
         configureDCSync0(false, 0, 0);
      }
   }
}
