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
   
   private MailbusDirection direction;
   
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
         
         slave.writeSDO(pdoConfigurationIndex, 0x0, (short)PDOs.size());
         
         for(int i = 0; i < PDOs.size(); i++)
         {
            slave.writeSDO(pdoConfigurationIndex, i, (short) PDOs.get(i).getAddress());
         }
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
         direction = MailbusDirection.RXPDO;
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
