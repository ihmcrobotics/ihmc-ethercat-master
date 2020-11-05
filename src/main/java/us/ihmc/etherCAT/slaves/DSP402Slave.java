package us.ihmc.etherCAT.slaves;

import us.ihmc.etherCAT.javalution.Struct.Unsigned16;
import us.ihmc.etherCAT.master.Slave;

/**
 * Implementation of the DSP402 state machine. Used in electric motor amplifiers. Tested on Copley and Elmo drives. 
 * 
 * @author Jesper Smith
 *
 */
public abstract class DSP402Slave extends Slave
{
   
   private static final int STATUSWORD_MASK = 0x8F; 
   private static final int CONTROLWORD_MFG_BITS_MASK = 0xFF70;

   private static final int noAction = Integer.parseInt("000", 2);
   private static final int shutdown = Integer.parseInt("110", 2);
   private static final int switchon = Integer.parseInt("0111", 2);
   private static final int disableVoltage = Integer.parseInt("0101", 2);
   private static final int enableOperation = Integer.parseInt("1111", 2);
   private static final int quickStopActive = Integer.parseInt("1011", 2);
   private static final int disableOp = Integer.parseInt("0111", 2);
   private static final int faultReset = Integer.parseInt("10000000", 2);
   private static final int STATUS_NOTREADYTOSWITCHON = 0x0;
   private static final int MASK_NOTREADYTOSWITCHON = 0x4F;
   private static final int STATUS_SWITCHONDISABLED = 0x40;
   private static final int MASK_SWITCHONDISABLED = 0x4F;
   private static final int STATUS_READYTOSWITCHON = 0x21;
   private static final int MASK_READYTOSWITCHON = 0x6F;
   private static final int STATUS_SWITHON = 0x23;
   private static final int MASK_SWITCHON = 0x6F;
   private static final int STATUS_OPERATIONENABLED = 0x27;
   private static final int MASK_OPERATIONENABLED = 0x6F;
   private static final int STATUS_QUICKSTOPACTIVE = 0x7;
   private static final int MASK_QUICKSTOPACTIVE = 0x4F;
   private static final int STATUS_FAULT = 0x8;
   private static final int MASK_FAULT = 0x4F;
 
   public enum ControlWord
   {
      NOACTION, SHUTDOWN, SWITCHON, DISABLEVOLTAGE, ENABLEOPERATION, QUICKSTOPACTIVE, DISABLEOP, FAULTRESET
   }

   public enum StatusWord
   {
      NOTREADYTOSWITCHON, SWITCHONDISABLED, READYTOSWITCHON, SWITCHEDON, OPERATIONENABLE, QUICKSTOPACTIVE, FAULT
   }

   private boolean enableDrive = false;
   private ControlWord command = null;

   
   
   public DSP402Slave(int vendor, int productCode, int alias, int position)
   {
      super(vendor, productCode, alias, position);
   }


   private void setControlWord(ControlWord control)
   {
      int cmd = 0;
      switch (control)
      {
      case NOACTION:
         cmd = noAction;
         break;
      case SHUTDOWN:
         cmd = shutdown;
         break;
      case SWITCHON:
         cmd = switchon;
         break;
      case DISABLEVOLTAGE:
         cmd = disableVoltage;
         break;
      case ENABLEOPERATION:
         cmd = enableOperation;
         break;
      case QUICKSTOPACTIVE:
         cmd = quickStopActive;
         break;
      case DISABLEOP:
         cmd = disableOp;
         break;
      case FAULTRESET:
         cmd = faultReset;
         break;
      default:
         throw new RuntimeException("Invalid command");
      }
   
      Unsigned16 controlWord = getControlWordPDOEntry();
      
      int mfgBits = controlWord.get() & CONTROLWORD_MFG_BITS_MASK;
      controlWord.set(cmd | mfgBits);
   
   }
   
   public StatusWord getStatus()
   {
      int statusWord = getStatusWordPDOEntry().get() & STATUSWORD_MASK;
      
      StatusWord state = StatusWord.NOTREADYTOSWITCHON;   
      
      if ((statusWord & MASK_NOTREADYTOSWITCHON) == STATUS_NOTREADYTOSWITCHON)
      {
         state = StatusWord.NOTREADYTOSWITCHON;
      } 
      else if ((statusWord & MASK_SWITCHONDISABLED) == STATUS_SWITCHONDISABLED)
      {
         state = StatusWord.SWITCHONDISABLED;
      } 
      else if ((statusWord & MASK_READYTOSWITCHON) == STATUS_READYTOSWITCHON)
      {
         state = StatusWord.READYTOSWITCHON;
      } 
      else if ((statusWord & MASK_SWITCHON) == STATUS_SWITHON)
      {
         state = StatusWord.SWITCHEDON;
      } 
      else if ((statusWord & MASK_OPERATIONENABLED) == STATUS_OPERATIONENABLED)
      {
         state = StatusWord.OPERATIONENABLE;
      } 
      else if ((statusWord & MASK_QUICKSTOPACTIVE) == STATUS_QUICKSTOPACTIVE)
      {
         state = StatusWord.QUICKSTOPACTIVE;
      } 
      else if ((statusWord & MASK_FAULT) == STATUS_FAULT)
      {
         state = StatusWord.FAULT;
      }
   
      return state;
   }

   public void doStateControl()
   {
      final StatusWord state = getStatus();
            
      
      switch (state) {
         case NOTREADYTOSWITCHON: {
            break;
         }
         case SWITCHONDISABLED: {
            
            if(command == ControlWord.FAULTRESET)
            {
               command = ControlWord.NOACTION;
            }
            else if(enableDrive)
            {
               command = ControlWord.SHUTDOWN;
            }
            
            break;
         }
         case READYTOSWITCHON: {
            command = ControlWord.SWITCHON;
            break;
         }
         case SWITCHEDON: {
            if(enableDrive)
            {
               command = ControlWord.ENABLEOPERATION;
            }
            else
            {
               command = ControlWord.DISABLEVOLTAGE;
            }
            break;
         }
         case OPERATIONENABLE: {
   
            if(!enableDrive)
               command = ControlWord.DISABLEVOLTAGE;
            
   
            break;
         }
         case QUICKSTOPACTIVE: {
   
            break;
         }
         case FAULT: {
   
            if(command == ControlWord.FAULTRESET)
            {
               command = ControlWord.NOACTION;
            }
            else
            {
//               System.err.println("Drive fault detected, resetting");
               command = ControlWord.FAULTRESET;
            }
            enableDrive = false;
            
            break;
         }
         default: {
            throw new RuntimeException("Unknown state");
         }
         
      }
      if(command != null)
         setControlWord(command);
      
   }
   
   public ControlWord getCurrentControlword()
   {
      return command;
   }

   public void enableDrive()
   {
      enableDrive = true;
   }

   public void disableDrive()
   {
      enableDrive = false;
   }

   public void setEnableDrive(boolean enable)
   {
      enableDrive = enable;
   }

   public boolean isDriveOperational()
   {
      return getStatus() == StatusWord.OPERATIONENABLE;
   }
   
   public boolean isReady()
   {
      int statusWord = getStatusWordPDOEntry().get();
      return isOperational() && statusWord != 0 && getStatus() != StatusWord.NOTREADYTOSWITCHON;
   }

   protected abstract Unsigned16 getStatusWordPDOEntry();
   protected abstract Unsigned16 getControlWordPDOEntry();

   
   @Override
   protected void shutdown()
   {
      disableDrive();
      doStateControl();
   }
   
   @Override 
   protected boolean hasShutdown()
   {
      return !isDriveOperational();
   }
   
}