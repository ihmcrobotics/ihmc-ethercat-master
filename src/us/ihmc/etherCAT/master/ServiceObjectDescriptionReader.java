package us.ihmc.etherCAT.master;

import us.ihmc.soem.generated.ec_ODlistt;
import us.ihmc.soem.generated.ec_datatype;
import us.ihmc.soem.generated.ecx_contextt;
import us.ihmc.soem.generated.soem;

/**
 * Helper class to CoE read SDO service object descriptions
 * 
 * @author jesper
 *
 */
public class ServiceObjectDescriptionReader
{

   private final ecx_contextt context;
   private final ec_ODlistt odList = new ec_ODlistt();

   private boolean read = false;

   /**
    * Constructor for construction using a Master
    * 
    * @param context
    */
   public ServiceObjectDescriptionReader(Master master)
   {
      this.context = master.getContext();
   }

   /**
    * Constructor for construction using a raw context
    * 
    * @param context
    */
   ServiceObjectDescriptionReader(ecx_contextt context)
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
   public boolean readObjectDescription(Slave slave, int objectIndex)
   {
      int index = slave.getSlaveIndex();
      return readObjectDescription(index, objectIndex);
   }

   /** 
    * Read a single OE object. 
    * 
    * @param slave SlaveIndex on the EtherCAT bus to read from
    * @param objectIndex Index of the OE object
    * @param objectSubindex SubIndex of the OE object
    * @return true if read is successful
    */
   public boolean readObjectDescription(int slaveIndex, int objectIndex)
   {
      odList.setSlave(slaveIndex);
      soem.uint16Array_setitem(odList.getIndex(), 0, objectIndex);

      int wkc = soem.ecx_readODdescription(context, 0, odList);

      if (wkc > 0)
      {
         read = true;
      }
      else
      {
         read = false;
      }
      
      return read;
   }

   private void testValidRead()
   {
      if (!read)
      {
         throw new RuntimeException("No data read. Call readObjectDescription() first and check if it returns true.");
      }
   }

   /**
    * Get index. Run after valid readObjectDescription
    * 
    * @return index. See EtherCAT specification
    */
   public int getIndex()
   {
      testValidRead();
      return soem.uint16Array_getitem(odList.getIndex(), 0);
   }

   /**
    * Get the DataType of the entry read by readObjectDescription.
    * 
    * @return DataType or null if invalid
    */
   public ec_datatype getDataType()
   {
      testValidRead();
      try
      {
         int dataType = soem.uint16Array_getitem(odList.getDataType(), 0);
         return ec_datatype.swigToEnum(dataType);
      }
      catch (IllegalArgumentException e)
      {
         return null;
      }
   }

   /**
    * Get the DataType as string value of the entry read by readObjectDescription.
    * 
    * @return DataType or raw value if invalid
    */
   public String getDataTypeAsString()
   {
      testValidRead();
      int dataType = soem.uint16Array_getitem(odList.getDataType(), 0);
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
    * Get the object code returned by readObjectDescription
    * 
    * @return Object code
    */
   public int getObjectCode()
   {
      testValidRead();
      return soem.uint8Array_getitem(odList.getObjectCode(), 0);
   }
   
   /**
    * Get maximum number of subindexes for each index returned by readObjectDescription
    * 
    * @return maximum number of subindexes
    */
   public int getObjectAccess()
   {
      testValidRead();
      return soem.uint8Array_getitem(odList.getMaxSub(), 0);

   }

   
   /**
    * Get the name of the object from the last entry read by readObjectDescription
    * 
    * @return name
    */
   public String getName()
   {
      testValidRead();
      return soem.ecx_odlist_name(odList, 0);
   }
}
