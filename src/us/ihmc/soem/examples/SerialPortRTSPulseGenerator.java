package us.ihmc.soem.examples;

import java.io.IOException;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

public class SerialPortRTSPulseGenerator
{
   
   private final CommPortIdentifier identifier;
   private final SerialPort serial;
   private final static int OPEN_TIMEOUT = 2000;

   public SerialPortRTSPulseGenerator(String port)
   {
      if ((System.getProperty("os.name").toLowerCase().indexOf("linux") != -1))
      {
         System.setProperty("gnu.io.rxtx.SerialPorts", port);
      }
      try
      {
         this.identifier = CommPortIdentifier.getPortIdentifier(port);
         CommPort commPort = identifier.open(getClass().getSimpleName(), OPEN_TIMEOUT); 
         if(commPort instanceof SerialPort)
         {
            serial = (SerialPort) commPort;
            serial.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
         }
         else
         {
            throw new IOException("Port is not a serial port");
         }
      }
      catch (NoSuchPortException | PortInUseException | IOException | UnsupportedCommOperationException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   
   public void setRTS(boolean RTS)
   {
      serial.setRTS(RTS);
   }
}
