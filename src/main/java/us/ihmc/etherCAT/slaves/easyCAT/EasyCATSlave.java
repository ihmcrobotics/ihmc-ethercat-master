package us.ihmc.etherCAT.slaves.easyCAT;

import java.io.IOException;

import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

/**
 * Created by Olger Siebinga & Koen Kramer on 3/22/16.
 * Can be used to create and communicate with an EasyCAT slave which is connected to a (programmed) Arduino
 */
public class EasyCATSlave extends Slave
{
   protected int easyCATSlaveFrameLength;
   
   private class RPDO_1600 extends RxPDO
   {

      private final Unsigned8[] data;

      protected RPDO_1600()
      {
         super(0x1600);
         data = array(new Unsigned8[easyCATSlaveFrameLength]);
      }
   }

   private class TPDO_1a00 extends TxPDO
   {
      private final Unsigned8[] data;

      protected TPDO_1a00()
      {
         super(0x1A00);
         data  = array(new Unsigned8[easyCATSlaveFrameLength]);
      }
   }

   //receive and transmit PDOs
   private final RPDO_1600 RPDO_x1600;
   private final TPDO_1a00 TPDO_x1A00;

   public EasyCATSlave(long vendorID, long productCode, int easyCATSlaveFrameLength, int alias, int ringPosition) throws IOException
   {
      super(vendorID, productCode, alias, ringPosition);
      this.easyCATSlaveFrameLength = easyCATSlaveFrameLength;

      RPDO_x1600 = new RPDO_1600();
      TPDO_x1A00 = new TPDO_1a00();
      
      registerSyncManager(new SyncManager(0, true));
      registerSyncManager(new SyncManager(1, true));
      
      sm(0).registerPDO(RPDO_x1600);
      sm(1).registerPDO(TPDO_x1A00);
   }

   //get desired TxPDOEntry values
   public void getTransmitBytes(int[] arrayToPack, int startIndex, int endIndex) //new method. should replace getTransmitBytes(int startIndex, int endIndex)
   {
      if ((startIndex < 0) || (endIndex > (easyCATSlaveFrameLength - 1)) || (startIndex > endIndex) || ((endIndex - startIndex) + 1 != arrayToPack.length))
      {
         throw new RuntimeException("Invalid method parameters");
      }
      for (int i = startIndex; i <= endIndex; i++)
      {
         arrayToPack[i] = TPDO_x1A00.data[i].get();
      }
   }

   //set first values.length number of RxPDOEntry values
   public void setReceiveBytes(int[] values)
   {
      if ((values == null) || (values.length > easyCATSlaveFrameLength))
      {
         throw new RuntimeException("invalid argument dimensions. Must be between 1 and 32");
      }

      for (int i = 0; i < values.length; i++)
      {
         RPDO_x1600.data[i].set((short) values[i]);
      }

   }
}
