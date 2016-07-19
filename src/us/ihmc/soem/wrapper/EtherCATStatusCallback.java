package us.ihmc.soem.wrapper;

import java.util.HashMap;

public class EtherCATStatusCallback
{
   private static final boolean TRACE = false;
   private static final long SYNC_MESSAGES_INTERVAL = 5000000000l;

   public enum TRACE_EVENT
   {
      FAST_IRQ("Setting up fast IRQ"),
      CREATE_CONTEXT("Creating context"),
      OPEN_INTERFACE("Opening interface"),
      CONFIGURING_SLAVES("Configured slaves"),
      DC_ENABLED("DC Enabled"),
      DC_DISABLED("DC Disabled"),
      INITIALIZING_SLAVES("Initializing slaves"),
      ALLOCATE_IOMAP("Allocating IO Map"),
      LINK_BUFFERS("Linking buffers to slave data"),
      CONFIGURE_TXRX("Sending first datagram"),
      CONFIGURE_COMPLETE("Configuration complete"),
      WAIT_SHUTDOWN("Waiting for EtherCAT Controller to stop"),
      STOP_HOUSEHOLDER("Shutting down controller thread"),
      SWITCH_PREOP("Switching slaves to PRE-OP state"),
      CLEANUP_SLAVES("Cleanup slaves"),
      SWITCH_TO_INIT("Switching slaves to INIT state"),
      CONFIGURE_DC("Configuring DC settings"),
      CLEAR_PDOS("Clearing PDO configuration"),
      WRITE_PDOS("Writing PDO entries"),
      WRITE_PDO_SIZE("Writing Number of PDO entries");

      private final String msg;

      private TRACE_EVENT(String msg)
      {
         this.msg = msg;
      }

      public String getMessage()
      {
         return msg;
      }
   }

   public void trace(TRACE_EVENT event)
   {
      if (TRACE)
      {
         System.out.println("[" + System.nanoTime() + "] Master: " + event.getMessage());
      }
   }

   public void trace(Slave slave, TRACE_EVENT event)
   {

   }

   public void trace(SyncManager syncManager, Slave slave, TRACE_EVENT event)
   {
      if (TRACE)
      {
         System.out.println("[" + System.nanoTime() + "] " + slave +" sm("+syncManager.getIndex() + "): " + event.getMessage());
      }
   }

   public void notifyStateChange(Slave slave, Slave.State previousState, Slave.State currentState)
   {
      if(TRACE)
      {
         System.out.println("Slave " + slave + " changed state from " + previousState + " to " + currentState);
      }

      switch (currentState)
      {
      case BOOT:
         break;
      case INIT:
         break;
      case PRE_OP:
         break;
      case PRE_OPERR:
         System.err.println(slave + " in PREOP+ERR. " + slave.getALStatusMessage());
         break;
      case SAFE_OP:
         break;
      case SAFE_OPERR:
         System.err.println(slave + " in SAFEOP+ERR. " + slave.getALStatusMessage());
         break;
      case OP:
         break;
      case OFFLINE:
         break;

      }
   }
   
   private final HashMap<Slave, Long> printedSlaveSyncOffsetMessages = new HashMap<>();
   public void reportDCSyncWaitTime(Slave slave, long runTime, int dcOffset)
   {
      if(!printedSlaveSyncOffsetMessages.containsKey(slave))
      {
         printedSlaveSyncOffsetMessages.put(slave, 0l);
      }
      
      long cycle = runTime / SYNC_MESSAGES_INTERVAL;
      long printed = printedSlaveSyncOffsetMessages.get(slave).longValue();
      
      if(cycle != printed)
      {
         System.err.println("[" + System.nanoTime() + "] " + slave.toString() + ": DC Clock not synchronized or slave refused OP mode for " + (runTime/1000000) + "ms. Current offset is " + dcOffset + "ns. AL Status: " + slave.getALStatusMessage());
         printedSlaveSyncOffsetMessages.put(slave, cycle);
      }
   }

   private long printedMasterThreadStableRateMessages = 0; 
   public void reportMasterThreadStableRateTime(long runTime, long jitterEstimate)
   {
      long cycle = runTime / SYNC_MESSAGES_INTERVAL;
      if(cycle != printedMasterThreadStableRateMessages)
      {
         System.err.println("[" + System.nanoTime() + "] Master thread not converged to stable rate for " + (runTime/1000000) + "ms. Current jitter estimate is " + jitterEstimate + "ns.");
         printedMasterThreadStableRateMessages = cycle;
      }
   }

}
