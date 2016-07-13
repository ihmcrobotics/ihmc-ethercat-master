package us.ihmc.soem.slaves;

import us.ihmc.soem.wrapper.Slave;
import us.ihmc.soem.wrapper.SyncManager;
import us.ihmc.soem.wrapper.TxPDO;

public class EL3314 extends Slave
{
   
   
   public class Input extends TxPDO
   {
      
      
      protected Input(int address)
      {
         super(address);
      }
      
      
      
      BooleanField underrange = new BooleanField();
      BooleanField overrange = new BooleanField();
      Member limit1 = new Bit2();
      Member limit2 = new Bit2();
      BooleanField error = new BooleanField();
      Bit7 gap = new Bit7();
      BooleanField txPDOState = new BooleanField();
      BooleanField txPDOToggle = new BooleanField();
      Signed16 value = new Signed16();
      
      
   };
   
   private final Input in1 = new Input(0x1a00);
   private final Input in2 = new Input(0x1a01);
   private final Input in3 = new Input(0x1a02);
   private final Input in4 = new Input(0x1a03);
   
   public EL3314(int aliasAddress, int configAddress)
   {
      super(aliasAddress, configAddress);
      
      for(int i = 0; i < 4; i++)
      {
         registerSyncManager(new SyncManager(i, true));
      }

      sm(3).registerPDO(in1);
      sm(3).registerPDO(in2);
      sm(3).registerPDO(in3);
      sm(3).registerPDO(in4);
      
      System.out.println("PDO SIZE " + in1.size());
   }
   
   public int getIn1()
   {
      return in1.value.get();
   }
   public int getIn2()
   {
      return in2.value.get();
   }
   public int getIn3()
   {
      return in3.value.get();
   }
   public int getIn4()
   {
      return in4.value.get();
   }
}
