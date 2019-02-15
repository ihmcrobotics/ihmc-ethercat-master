package us.ihmc.etherCAT.examples;

import static java.lang.System.out;

import java.awt.Color;
import java.awt.FlowLayout;
import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Enumeration;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import us.ihmc.etherCAT.slaves.EasyCATSlave;
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
public class EasyCATExample extends EtherCATRealtimeThread {

	private final EasyCATSlave wirelessButtonsSlave = new EasyCATSlave(0, 0);
	private final EasyCATSlave encoderSlave = new EasyCATSlave(0, 1);

	private JLabel button1Label;
	private JLabel button2Label;
	private JLabel button3Label;
	private JLabel button4Label;
	private JLabel analogLabel1;

	private static String networkCard = "enp6s0";

	private JFrame frame;

	private static volatile boolean button1 = false;
	private static volatile boolean button2 = false;
	private static volatile boolean button3 = false;
	private static volatile boolean button4 = false;
	private static volatile int temperature = 0;

	private Thread uiButtonsThread;

	public EasyCATExample() throws IOException {
		super(networkCard, new PriorityParameters(PriorityParameters.getMaximumPriority()),
				new MonotonicTime(0, 1000000), true, 100000);

		System.out.println("Starting EtherCAT");
		System.out.println("Registering Slave...");

		registerSlave(wirelessButtonsSlave);
		registerSlave(encoderSlave);

		// Run the UI on its own thread
		Thread thread = new Thread("UI Thread") {

			public void run() {
				setupUI();
			}

		};

		thread.start();

	}

	@Override
	protected void workingCounterMismatch(int expected, int actual) {

		System.out.println("Working counter mismatch!!!");

	}

	int counter = 0;

	@Override
	protected void deadlineMissed() {

		System.out.println("Deadlines missed so far: " + counter);
		counter++;

	}

	@Override
	protected void doControl() {

		// extract frame data from each slave
		int[] frameData = new int[32];
		wirelessButtonsSlave.getTransmitBytes(frameData, 0, 31);
		parseButtonInputs(frameData);

		frameData = new int[32];
		encoderSlave.getTransmitBytes(frameData, 0, 31);
		parseAnalogInputs(frameData);


		synchronized (uiButtonsThread) {

			uiButtonsThread.notify();

		}

	}

	@Override
	protected void doReporting() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void datagramLost() {
		System.out.println("DATAGRAM Lost!!");

	}

	public static void main(String args[]) throws IOException {
		/*
		 * Enumeration<NetworkInterface> nets; try { nets =
		 * NetworkInterface.getNetworkInterfaces(); for (NetworkInterface netint :
		 * Collections.list(nets)) displayInterfaceInformation(netint);
		 * 
		 * } catch (SocketException e) { // TODO Auto-generated catch block
		 * e.printStackTrace(); }
		 */
		EasyCATExample easyCat = new EasyCATExample();
		easyCat.start();
		easyCat.join();

	}

	static void displayInterfaceInformation(NetworkInterface netint) throws SocketException {
		out.printf("Display name: %s\n", netint.getDisplayName());
		out.printf("Name: %s\n", netint.getName());
		Enumeration<InetAddress> inetAddresses = netint.getInetAddresses();
		for (InetAddress inetAddress : Collections.list(inetAddresses)) {
			out.printf("InetAddress: %s\n", inetAddress);
		}
		out.printf("\n");
	}

	// extract wanted bytes and parse them
	private void parseButtonInputs(int[] data) {

		if (data[2] == 0) {
			button1 = false;
		} else {
			button1 = true;
		}

		if (data[3] == 0) {
			button2 = false;
		} else {
			button2 = true;
		}

		if (data[4] == 0) {
			button3 = false;
		} else {
			button3 = true;
		}

		if (data[5] == 0) {
			button4 = false;
		} else {
			button4 = true;
		}
	}

	// extract wanted bytes and parse them
	private void parseAnalogInputs(int[] data) {
		temperature = (int) data[0];
	}

	public void updateUI() {

		button1Label.updateUI();
		button2Label.updateUI();
		button3Label.updateUI();
		button4Label.updateUI();

		if (button1) {
			button1Label.setBackground(Color.GREEN);

		} else {
			button1Label.setBackground(Color.GRAY);
		}

		if (button2) {
			button2Label.setBackground(Color.GREEN);

		} else {
			button2Label.setBackground(Color.GRAY);
		}

		if (button3) {
			button3Label.setBackground(Color.GREEN);

		} else {
			button3Label.setBackground(Color.GRAY);
		}

		if (button4) {
			button4Label.setBackground(Color.GREEN);

		} else {
			button4Label.setBackground(Color.GRAY);
		}

		analogLabel1.setText("   Analog #1:   " + temperature);

		frame.pack();

	}

	public void setupUI() {

		frame = new JFrame("CED - EtherCAT Test");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		JPanel panel = new JPanel();
		panel.setLayout(new FlowLayout());

		button1Label = new JLabel("  Button 1  ");
		panel.add(button1Label);

		button2Label = new JLabel("  Button 2  ");
		panel.add(button2Label);

		button3Label = new JLabel("  Button 3  ");
		panel.add(button3Label);

		button4Label = new JLabel("  Button 4  ");
		panel.add(button4Label);

		analogLabel1 = new JLabel("  Analog #1:      ");
		panel.add(analogLabel1);

		frame.add(panel);

		frame.pack();
		frame.setVisible(true);

		button1Label.setOpaque((true));
		button2Label.setOpaque((true));
		button3Label.setOpaque((true));
		button4Label.setOpaque((true));

		button1Label.setBackground(Color.GRAY);
		button2Label.setBackground(Color.GRAY);
		button3Label.setBackground(Color.GRAY);
		button4Label.setBackground(Color.GRAY);

		uiButtonsThread = new Thread("UI Buttons") {

			public void run() {

				while (true) {

					synchronized (uiButtonsThread) {

						try {
							uiButtonsThread.wait();
							updateUI();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				}
			}

		};

		uiButtonsThread.start();
	}
}
