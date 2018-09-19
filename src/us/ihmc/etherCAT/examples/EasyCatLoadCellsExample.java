package us.ihmc.etherCAT.examples;

import static java.lang.System.out;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Enumeration;

import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.etherCAT.slaves.EasyCATLoadCellSlave;
import us.ihmc.etherCAT.slaves.EasyCATVerticalPositionerSlave;
import us.ihmc.etherCAT.slaves.EasyCATVerticalPositionerSlaveWithButtons;
import us.ihmc.etherCAT.slaves.EasyCATWirelessButtonsSlave;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;

/**
 * This is an example for testing an EasyCAT module with the IHMC etherCAT
 * Master code stack. The EasyCAT module must be coupled with an arduino board
 * running the standard easyCAT demo (32 byte frames). Make sure the arduino is
 * powered and the etherCAT cable properly connected.
 * 
 * This demo will attempt to read 4 different digital bytes and 1 analog byte
 * from two different easyCat slaves and display the results graphically through
 * a simple swing-based UI. If only one slave is desired comment out the code
 * appropriately.
 * 
 * 
 * 
 * @author dduran
 *
 */
public class EasyCatLoadCellsExample extends EtherCATRealtimeThread
{

   private final EasyCATLoadCellSlave loadCellSlave1 = new EasyCATLoadCellSlave(151, 0);
   private final EasyCATLoadCellSlave loadCellSlave2 = new EasyCATLoadCellSlave(152, 0);
   private final EasyCATVerticalPositionerSlaveWithButtons verticalPositonerWithButtons = new EasyCATVerticalPositionerSlaveWithButtons(0, 0);
   private final EasyCATVerticalPositionerSlave verticalPositoner = new EasyCATVerticalPositionerSlave(0, 0);

   private final EasyCATWirelessButtonsSlave wirelessButtonSlave = new EasyCATWirelessButtonsSlave(0, 0);

   private static String networkCard = "enp4s0";

   public EasyCatLoadCellsExample() throws IOException
   {
      super(networkCard, new PriorityParameters(PriorityParameters.getMaximumPriority()), new MonotonicTime(0, 1000000), true, 100000);

      System.out.println("Starting EtherCAT");
      System.out.println("Registering Slave...");

      //		registerSlave(loadCellSlave1);
      //		registerSlave(loadCellSlave2);
      registerSlave(verticalPositonerWithButtons);
      //      registerSlave(verticalPositoner);

      //      registerSlave(wirelessButtonSlave);
   }

   @Override
   protected void workingCounterMismatch(int expected, int actual)
   {

      System.out.println("Working counter mismatch!!!");
      System.out.println("Expected: " + expected);
      System.out.println("Actual: " + actual);
   }

   int counter = 0;

   @Override
   protected void deadlineMissed()
   {

      System.out.println("Deadlines missed so far: " + counter);
      counter++;

   }

   @Override
   protected void doControl()
   {

      // extract frame data from each slave

      verticalPositonerWithButtons.processDataFromVerticalPositioner();
      System.out.println("Position: " + verticalPositonerWithButtons.getCurrentPositionInMeters() + "   Ld: "
            + verticalPositonerWithButtons.isLowerLimitSwitchPressed() + "   Lu: " + verticalPositonerWithButtons.isUpperLimitSwitchPressed() + "   Md: "
            + verticalPositonerWithButtons.isManualDownButtonPressed() + "   Mu: " + verticalPositonerWithButtons.isManualUpButtonPressed());

      verticalPositonerWithButtons.setDesiredPositionInMeters(0.80);
      verticalPositonerWithButtons.processOutputCommands();

      //          System.out.println("Position: " + verticalPositoner.getCurrentPositionInMeters() + "   Ld: " + verticalPositoner.isLowerLimitSwitchPressed() + "   Lu: "
      //                + verticalPositoner.isUpperLimitSwitchPressed());
      //            

      //      verticalPositonerWithButtons.processDataFromVerticalPositioner();

      //      verticalPositoner.setDesiredPositionInMeters(0.80);
      //      verticalPositoner.processOutputCommands();

      //      wirelessButtonSlave.processData();
      //      System.out.println(wirelessButtonSlave.getButtonStates()[0] + "    " + wirelessButtonSlave.getButtonStates()[1] + "    "
      //            + wirelessButtonSlave.getButtonStates()[2] + "    " + wirelessButtonSlave.getButtonStates()[3]);
   }

   @Override
   protected void doReporting()
   {
      // TODO Auto-generated method stub

   }

   @Override
   protected void datagramLost()
   {
      System.out.println("DATAGRAM Lost!!");

   }

   public static void main(String args[]) throws IOException
   {
      /*
       * Enumeration<NetworkInterface> nets; try { nets =
       * NetworkInterface.getNetworkInterfaces(); for (NetworkInterface netint :
       * Collections.list(nets)) displayInterfaceInformation(netint); } catch
       * (SocketException e) { // TODO Auto-generated catch block
       * e.printStackTrace(); }
       */
      EasyCatLoadCellsExample easyCat = new EasyCatLoadCellsExample();
      easyCat.start();
      easyCat.join();

   }

   static void displayInterfaceInformation(NetworkInterface netint) throws SocketException
   {
      out.printf("Display name: %s\n", netint.getDisplayName());
      out.printf("Name: %s\n", netint.getName());
      Enumeration<InetAddress> inetAddresses = netint.getInetAddresses();
      for (InetAddress inetAddress : Collections.list(inetAddresses))
      {
         out.printf("InetAddress: %s\n", inetAddress);
      }
      out.printf("\n");
   }

}
