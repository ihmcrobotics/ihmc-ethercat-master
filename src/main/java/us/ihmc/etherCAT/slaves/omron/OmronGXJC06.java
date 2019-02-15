package us.ihmc.etherCAT.slaves.omron;

import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

public class OmronGXJC06
{

   public static final int vendorID = 0x00000083;
   public static final int productID = 0x00000081;
   public static final int subProductID = 0x00000082;

   private class TxPDO1A00 extends TxPDO
   {
      private TxPDO1A00()
      {
         super(0x1a00);
      }

      public Unsigned16 ID = new Unsigned16();
   }

   private final TxPDO1A00 txPDO1A00 = new TxPDO1A00();

   private final Junction main;
   private final Junction sub;

   private final int omronJunctionAlias;
   private final int omronSubJunctionAlias;

   public OmronGXJC06(int omronJunctionAlias, int position, int omronSubJunctionAlias, int subPosition)
   {
      this.omronJunctionAlias = omronJunctionAlias;
      this.omronSubJunctionAlias = omronSubJunctionAlias;

      this.main = new Junction(vendorID, productID, omronJunctionAlias, position);
      this.sub = new Junction(vendorID, subProductID, omronSubJunctionAlias, subPosition);

      SyncManager sm1 = new SyncManager(1, false);
      sm1.registerPDO(txPDO1A00);
      main.registerSyncManager(sm1);

   }

   public boolean isOperational()
   {
      return main.isOperational() && sub.isOperational();
   }

   public Slave getMainJunction()
   {
      return main;
   }

   public Slave getSubJunction()
   {
      return sub;
   }

   public int getID()
   {
      return txPDO1A00.ID.get();
   }


   public int getPort2Alias()
   {
      return omronJunctionAlias;
   }

   public int getPort3Alias()
   {
      return omronJunctionAlias;
   }

   public int getPort4Alias()
   {
      return omronSubJunctionAlias;
   }

   public int getPort5Alias()
   {
      return omronSubJunctionAlias;
   }

   public int getPort6Alias()
   {
      return omronSubJunctionAlias;
   }

   private class Junction extends Slave
   {

      public Junction(int vendor, int productCode, int aliasAddress, int position)
      {
         super(vendor, productCode, aliasAddress, position);
      }


   }

}

