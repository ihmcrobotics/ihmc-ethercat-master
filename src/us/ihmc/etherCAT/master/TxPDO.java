package us.ihmc.etherCAT.master;


/**
 * PDO object to describe data Transmitted (Tx) by the slave
 * 
 * To use this class, extend it and define PDO entries using the following data types 
 * 
 * BooleanField boolean = new BooleanField();
 * Bit[2-7] gap = new Bit[2-7]();
 * Unsigned8 uint8 = new Unsigned8();
 * Signed8 int8 = new Signed8();
 * Unsigned16 uint16 = new Unsigned16();
 * Signed16 int16 = new Signed16();
 * Unsigned32 uint32 = new Unsigned32();
 * Signed32 int32 = new Signed32();
 * Signed64 int64 = new Signed64();
 * Float32 float32 = new Float32();
 * Float64 float64 = new Float64();
 * 
 * 
 * Make sure the data fields correspond to the order and size of the slave specification
 *  
 * @author Jesper Smith
 *
 */
public abstract class TxPDO extends PDO
{

   /**
    * Create new TxPDO 
    * 
    * @param address Address of the PDO
    */
   protected TxPDO(int address)
   {
      super(address);
   }

   
   
}
