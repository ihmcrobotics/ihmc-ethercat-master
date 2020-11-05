package us.ihmc.etherCAT.dataStructures;

/**
 * This class represents an Union, usable within an EtherCAT struct.
 * 
 * @author Jesper Smith
 *
 */
public class EtherCATUnion extends EtherCATStruct
{
   @Override
   public final boolean isUnion()
   {
      return true;
   }


}
