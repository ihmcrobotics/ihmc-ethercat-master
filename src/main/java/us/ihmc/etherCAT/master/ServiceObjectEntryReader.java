package us.ihmc.etherCAT.master;

import us.ihmc.soem.generated.ec_ODlistt;
import us.ihmc.soem.generated.ec_OElistt;
import us.ihmc.soem.generated.ec_datatype;
import us.ihmc.soem.generated.ecx_context;
import us.ihmc.soem.generated.soem;

/**
 * Helper class to CoE read SDO service object entries
 * 
 * @author jesper
 *
 */
public class ServiceObjectEntryReader
{

   private final ecx_context context;
   private final ec_ODlistt odList = new ec_ODlistt();
   private final ec_OElistt oeList = new ec_OElistt();

   private int lastReadSubIndex = -1;

   /**
    * Constructor for construction using a Master
    * 
    * @param context
    */
   public ServiceObjectEntryReader(Master master)
   {
      this.context = master.getContext();
   }

   /**
    * Constructor for construction using a raw context
    * 
    * @param context
    */
   ServiceObjectEntryReader(ecx_context context)
   {
      this.context = context;
   }

   /** 
    * Read a single OE object. 
    * 
    * @param slave Slave to read from
    * @param objectIndex Index of the OE object
    * @param objectSubindex Subindex of the OE object
    * @return true if read is successful
    */
   public boolean readSingleOE(Slave slave, int objectIndex, int objectSubindex)
   {
      int index = slave.getSlaveIndex();
      return readSingleOE(index, objectIndex, objectSubindex);
   }

   /** 
    * Read a single OE object. 
    * 
    * @param slave SlaveIndex on the EtherCAT bus to read from
    * @param objectIndex Index of the OE object
    * @param objectSubindex SubIndex of the OE object
    * @return true if read is successful
    */
   public boolean readSingleOE(int slaveIndex, int objectIndex, int objectSubindex)
   {
      odList.setSlave(slaveIndex);
      soem.uint16Array_setitem(odList.getIndex(), 0, objectIndex);
      oeList.setEntries(0);
      int wkc = soem.ecx_readOEsingle(context, 0, (short) objectSubindex, odList, oeList);

      if (wkc > 0 && oeList.getEntries() > 0)
      {
         lastReadSubIndex = objectSubindex;
         return true;
      }
      else
      {
         lastReadSubIndex = -1;
         return false;
      }
   }

   private void testValidRead()
   {
      if (lastReadSubIndex < 0)
      {
         throw new RuntimeException("No data read. Call readSingleOE() first and check if it returns true.");
      }
   }

   /**
    * Get value info. Returns data read using readSingleOE()
    * 
    * @return value info. See EtherCAT specification
    */
   public int getValueInfo()
   {
      testValidRead();
      return soem.uint8Array_getitem(oeList.getValueInfo(), lastReadSubIndex);
   }

   /**
    * Get the DataType of the entry read by readSingleOE.
    * 
    * @return DataType or null if invalid
    */
   public ec_datatype getDataType()
   {
      testValidRead();
      try
      {
         int dataType = soem.uint16Array_getitem(oeList.getDataType(), lastReadSubIndex);
         return ec_datatype.swigToEnum(dataType);
      }
      catch (IllegalArgumentException e)
      {
         return null;
      }
   }

   /**
    * Get the DataType as string value of the entry read by readSingleOE.
    * 
    * @return DataType or raw value if invalid
    */
   public String getDataTypeAsString()
   {
      testValidRead();
      int dataType = soem.uint16Array_getitem(oeList.getDataType(), lastReadSubIndex);
      try
      {
         return ec_datatype.swigToEnum(dataType).toString().replace("ECT_", "");
      }
      catch (IllegalArgumentException e)
      {
         return String.valueOf(dataType);
      }
   }
   
   /**
    * Get the bitlength returned from the last entry read by readSingleOE
    * 
    * @return Bit length
    */
   public int getBitLength()
   {
      testValidRead();
      return soem.uint16Array_getitem(oeList.getBitLength(), lastReadSubIndex);
   }
   
   /**
    * Get the object access from the last entry read by readSingleOE
    * 
    * @return Object access rights as integer
    */
   public int getObjectAccess()
   {
      testValidRead();
      return soem.uint16Array_getitem(oeList.getObjAccess(), lastReadSubIndex);

   }

   
   /**
    * Get the name of the object from the last entry read by readSingleOE
    * 
    * @return name
    */
   public String getName()
   {
      testValidRead();
      return soem.ecx_oelist_name(oeList, lastReadSubIndex);
   }
}
