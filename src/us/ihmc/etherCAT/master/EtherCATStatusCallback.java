package us.ihmc.etherCAT.master;

import java.nio.ByteBuffer;
import java.util.HashMap;

public class EtherCATStatusCallback
{
   private final boolean TRACE;
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
      WRITE_PDO_SIZE("Writing Number of PDO entries"),
      RECONFIG_TO_PREOP("Reconfiguring slave to PRE-OP"),
      RECONFIG_TO_SAFEOP("Reconfiguring slave to SAFE-OP"),
      RECOVER_SLAVE("Slave lost. Recovering slave"),
      RECOVERED_SLAVE("Sucessfully recovered slave"),
      SLAVE_FOUND("Slave found"),
      SLAVE_LOST("Slave lost"),
      READ_WATCHDOG_DIV("Reading watchdog division time"),
      WRITE_WATCHDOG_TIMEOUT("Writing watchdog timeout");

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

   public EtherCATStatusCallback(boolean trace)
   {
      this.TRACE = trace;
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
      if (TRACE)
      {
         System.out.println("[" + System.nanoTime() + "] " + slave + " " + event.getMessage());
      }
   }

   public void trace(SyncManager syncManager, Slave slave, TRACE_EVENT event)
   {
      if (TRACE)
      {
         System.out.println("[" + System.nanoTime() + "] " + slave + " sm(" + syncManager.getIndex() + "): " + event.getMessage());
      }
   }

   public void notifyStateChange(Slave slave, Slave.State previousState, Slave.State currentState)
   {
      if (TRACE)
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
      if (!printedSlaveSyncOffsetMessages.containsKey(slave))
      {
         printedSlaveSyncOffsetMessages.put(slave, 0l);
      }

      long cycle = runTime / SYNC_MESSAGES_INTERVAL;
      long printed = printedSlaveSyncOffsetMessages.get(slave).longValue();

      if (cycle != printed)
      {
         System.err.println("[" + System.nanoTime() + "] " + slave.toString() + ": DC Clock not synchronized or slave refused OP mode for " + (runTime / 1000000) + "ms. Current offset is " + dcOffset + "ns. AL Status: " + slave.getALStatusMessage());
         printedSlaveSyncOffsetMessages.put(slave, cycle);
      }
   }

   private long printedMasterThreadStableRateMessages = 0;

   public void reportMasterThreadStableRateTime(long runTime, long jitterEstimate)
   {
      long cycle = runTime / SYNC_MESSAGES_INTERVAL;
      if (cycle != printedMasterThreadStableRateMessages)
      {
         System.err.println("[" + System.nanoTime() + "] Master thread not converged to stable rate for " + (runTime / 1000000) + "ms. Current jitter estimate is " + jitterEstimate
               + "ns. Make sure to run a real time kernel. To increase maximum allowed jitter, use EtherCATRealtimeThread.setMaximumExecutionJitter()");
         printedMasterThreadStableRateMessages = cycle;
      }
   }

   public static String hex(long n)
   {
      return String.format("%2s", Long.toHexString(n)).replace(' ', '0');
   }

   public void notifySDOWrite(Slave slave, int index, int subindex, int wc, ByteBuffer buffer)
   {
      if (TRACE)
      {
         System.out.print("[" + System.nanoTime() + "] " + slave + " SDO Write " + Integer.toHexString(index) + ":" + Integer.toHexString(subindex) + "; wc: " + wc + "; size: " + buffer.position());
         System.out.print(" Data: ");
         for (int i = 0; i < buffer.position(); i++)
         {
            System.out.print(hex(buffer.get(i)));
         }
         System.out.println();
      }
   }

   public void notifyReadSDO(Slave slave, int index, int size, int subindex, int wc, ByteBuffer buffer)
   {
      if (TRACE)
      {
         System.out.print("[" + System.nanoTime() + "] " + slave + " SDO Read " + Integer.toHexString(index) + ":" + Integer.toHexString(subindex) + "; wc: " + wc);
         System.out.print(" Data: ");
         for (int i = 0; i < size; i++)
         {
            System.out.print(hex(buffer.get(i)));
         }
         System.out.println();
      }
   }

   public void pdoConfigurationError(Slave slave, int index, int pdoConfigurationIndex)
   {
      System.err.println("[" + System.nanoTime() + "] " + slave + " sm(" + index + "): Cannot configure PDO size on index " + Integer.toHexString(pdoConfigurationIndex) + ". Object is read-only");
   }

   public void pdoConfigurationError(Slave slave, int index, int pdoConfigurationIndex, int pdoConfigurationSubIndex, int pdoIndex)
   {
      System.err.println("[" + System.nanoTime() + "] " + slave + " sm(" + index + "): Cannot Write PDO " + Integer.toHexString(pdoIndex) + " configuration to index " + Integer.toHexString(pdoConfigurationIndex) + ":"
            + Integer.toHexString(pdoConfigurationSubIndex) + ". Object is read-only.");
   }

   public void notifyExpectedWorkingCounter(long expectedWorkingCounter)
   {
      if (TRACE)
      {
         System.out.println("[" + System.nanoTime() + "] Calculated expected working counter: " + expectedWorkingCounter);
      }
   }

   public void notifyWorkingCounterMismatch()
   {
      System.err.println("[" + System.nanoTime() + "] Working counter mismatch. Recovering slaves.");
   }

   public void notifyDCNotCapable()
   {
      System.err.println("[" + System.nanoTime() + "] Cannot configure distrubed clocks, running without.");
   }

   public void notifyWatchdogConfigurationError(Slave slave)
   {
      System.err.println("[" + System.nanoTime() + "] Cannot configure PDO watchdog timeout.");
   }

   public void notifyWatchdogConfiguration(Slave slave, int pdoWatchdogTimeout, int watchdogDiv, int watchdogPDORaw)
   {
      if(TRACE)
      {
         System.out.println("[" + System.nanoTime() + "] Configuring PDO watchdog to " + pdoWatchdogTimeout + " ns. Divisor " + watchdogDiv + ", setting to raw " + watchdogPDORaw);
      }
   }

}
