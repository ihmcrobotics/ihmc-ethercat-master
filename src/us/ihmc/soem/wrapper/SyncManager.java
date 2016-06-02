package us.ihmc.soem.wrapper;

import java.nio.ByteBuffer;
import java.util.ArrayList;

public class SyncManager
{
   public enum MailbusDirection 
   {
      TXPDO,
      RXPDO,
      UNKNOWN
   };
   
   private final ArrayList<PDO> PDOs = new ArrayList<>();
   private final boolean cconfigurePDOs;
   private final int index;
   
   private MailbusDirection direction = MailbusDirection.UNKNOWN;
   
   public SyncManager(int index, boolean configurePDOs)
   {
      this.index = index;
      this.cconfigurePDOs = configurePDOs;
   }
   
   void configure(Slave slave)
   {
      if(cconfigurePDOs)
      {
         int pdoConfigurationIndex = 0x1C10 + index;
         
         
         // Set the size of the SM configuration array to zero, allows writing to the elements
         slave.writeSDO(pdoConfigurationIndex, 0x0, 0);
         
         // Configure all the PDOs
         for(int i = 0; i < PDOs.size(); i++)
         {
            slave.writeSDO(pdoConfigurationIndex, i + 1, (short) PDOs.get(i).getAddress());
         }
         
         // Set the correct size of the SM array
         slave.writeSDO(pdoConfigurationIndex, 0x0, (byte) PDOs.size());
      }
   }

   public int getIndex()
   {
      return index;
   }

   
   public void registerPDO(RxPDO pdo)
   {
      switch(direction)
      {
      case TXPDO:
         throw new RuntimeException("Cannot register both Tx and Rx PDO entries on a single syncmanager");
      case UNKNOWN:
         direction = MailbusDirection.RXPDO;
      case RXPDO:
         PDOs.add(pdo);
         break;
      }
   }
   
   public void registerPDO(TxPDO pdo)
   {
      switch(direction)
      {
      case RXPDO:
         throw new RuntimeException("Cannot register both Tx and Rx PDO entries on a single syncmanager");
      case UNKNOWN:
         direction = MailbusDirection.TXPDO;
      case TXPDO:
         PDOs.add(pdo);
         break;
      }
   }
   
   public MailbusDirection getMailbusDirection()
   {
      return direction;
   }

   public int processDataSize()
   {
      int size = 0;
      for(int i = 0; i < PDOs.size(); i++)
      {
         size += PDOs.get(i).size();
      }
      return size;
   }

   public int linkBuffers(ByteBuffer ioMap, int inputOffset)
   {
      for(int i = 0; i < PDOs.size(); i++)
      {
         PDOs.get(i).setByteBuffer(ioMap, inputOffset);
         inputOffset += PDOs.get(i).size();
      }
      
      return inputOffset;
   }
}
