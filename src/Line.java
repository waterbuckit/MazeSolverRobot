import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.util.LinkedList;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.util.Delay;

public class Line {
	
	static BTConnection connection = null;
	static DataInputStream dis;
	static DataOutputStream dos;
	static int MAX_READ = 30;
	static byte[] buffer = new byte[MAX_READ];
	static int start;
	static int end;
	static LinkedList<Node> nodes;
	static DifferentialPilot pilot;
	static LightSensor ls;
	static double maxValue;	
	static double minValue;
	static final float defaultSpeed = 70;
	static int currentID;
	
	public static void main(String args[]) {
		pilot = new DifferentialPilot(56, 115, Motor.A, Motor.B);
		ls = new LightSensor(SensorPort.S1);
		pilot.setTravelSpeed(defaultSpeed);
		connectToPhone();
		setStartandEnd();
		calibrate();
		Button.ENTER.waitForPressAndRelease(); // when we would like to start
		Behavior[] behaviorList = {new Follow(), new BluetoothHandler(), new Stop()};
		Arbitrator arb = new Arbitrator(behaviorList);
		arb.start();
	}
	private static void setStartandEnd() {
		LCD.clear();
		LCD.drawString("Start: ", 0, 0);
		start = connection.read(buffer, MAX_READ);
		Delay.msDelay(1000);
		LCD.clear();
		LCD.drawString("End: ", 0, 0);
		end = connection.read(buffer, MAX_READ);
		Delay.msDelay(1000);
		LCD.clear();
		LCD.drawInt(start, 0, 1);
		LCD.drawInt(end, 0, 2);
		Delay.msDelay(1000);
	}
	private static void connectToPhone() {
		LCD.drawString("Waiting  ", 0, 0);
		Line.connection = Bluetooth.waitForConnection(0,  NXTConnection.RAW);
		LCD.drawString("Connected", 0, 0);
	}
//		byte[] buffer = new byte[MAX_READ];
//		
//		while (true) {
//			if (connection != null) {
//				if (connection.available() > 0) {
//					LCD.drawString("Chars read: ", 0, 2);
//					LCD.drawInt(connection.available(), 12, 2);
//					int read = connection.read(buffer, MAX_READ);
//					LCD.drawChar('[', 3, 3);
//					for (int index= 0 ; index < read ; index++) {						
//						LCD.drawChar((char)buffer[index], index + 4, 3);
//					}
//					LCD.drawChar(']', read + 4, 3);
//					connection.write(buffer, read);
//				}
//			}
//		
//	}
//	public static int convert(int readValue) {
//		return (int)((int)(readValue - minValue)/(maxValue - minValue) * 100);
//	}
	private static void calibrate() {
		LCD.clear();
		LCD.drawString("White", 1,1 );
		Button.ENTER.waitForPressAndRelease();
		ls.calibrateHigh();
		LCD.clear();
		LCD.drawString("Black", 1, 1);
		Button.ENTER.waitForPressAndRelease();
		ls.calibrateLow();
		LCD.clear();
	}	
	static boolean checkNodes(int val) {
		for(Node node : nodes) {
			if(node.getID() == val) {
				return true;
			}
		}
		return false;
	}
}

class Junction implements Behavior{

	@Override
	public boolean takeControl() {
		if(Line.currentID != 0) { // check this later because the byte read may be zero
			return true;
		}
		return false;
	}

	@Override
	public void action() {
		Line.pilot.stop();
		correct();
		
		Node current = null;
		if(!Line.checkNodes(Line.currentID)) {
			Line.nodes.add(new Node(Line.currentID));
			current = Line.nodes.get(Line.nodes.size()-1);
		}else {
			for(Node node : Line.nodes) {
				if(node.getID() == Line.currentID) {
					current = node;
				}
			}
		}
		if(current.getTimesVisited() == 4) {
			Line.pilot.rotate(-90);
			return;
		}else {
			
		}
		
	}

	@Override
	public void suppress() {
		
	}
	private void correct() {
		Line.pilot.travel(10000);
	}
	
}

class BluetoothHandler implements Behavior{
	
	@Override
	public boolean takeControl() {
		if(Line.connection != null && Line.connection.available() > 0) {
			return true;
		}
		return false;
	}

	@Override
	public void action() {
		LCD.drawString("Chars read: ", 0, 2);
		LCD.drawInt(Line.connection.available(), 12, 2);
		int read = Line.connection.read(Line.buffer, Line.MAX_READ);
		LCD.drawChar('[', 3, 3);
		for (int index= 0 ; index < read ; index++) {						
			LCD.drawChar((char)Line.buffer[index], index + 4, 3);
			
		}
		LCD.drawChar(']', read + 4, 3);
		Line.connection.write(Line.buffer, read);
		Line.currentID = read;
	}

	@Override
	public void suppress() {
	}
	
}
class Follow implements Behavior{

	boolean isSuppressed = false;
	
	@Override
	public boolean takeControl() {
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	public void action() {
		// TODO Auto-generated method stub
		if(isSuppressed) {
			return;
		}
		Line.pilot.forward();
		int val = Line.ls.readValue();
		Motor.A.setSpeed((val/100)*Line.defaultSpeed);
		Motor.B.setSpeed(Line.defaultSpeed - (val/100)*Line.defaultSpeed);
	}

	@Override
	public void suppress() {
		isSuppressed = true;
	}
	
	public void unsuppress() {
		isSuppressed = false;
	}
	
}
class Node {
	
	int timesVisited = 0;
	int id;
	
	public Node(int id) {
		this.id = id;
	}
	
	public int getID() {
		return this.id;
	}
	
	public int getTimesVisited() {
		return this.timesVisited;
	}
	
	public void incrementTimesVisited() {
		this.timesVisited++;
	}
}
class Stop implements Behavior{

	@Override
	public boolean takeControl() {
		return Button.ENTER.isDown();
	}

	@Override
	public void action() {
		System.exit(0);
	}

	@Override
	public void suppress() {
	}
	
}
