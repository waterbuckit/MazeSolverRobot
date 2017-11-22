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
	static int MAX_READ = 15;
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
	static Node current;
	static boolean look;
	static int turn; 
	
	public static void main(String args[]) {
		pilot = new DifferentialPilot(56, 115, Motor.A, Motor.B);
		ls = new LightSensor(SensorPort.S1,true);
		pilot.setTravelSpeed(defaultSpeed);
		nodes = new LinkedList<>();
		connectToPhone();
		setStartandEnd();
		calibrate();
		Button.ENTER.waitForPressAndRelease(); // when we would like to start
		Behavior[] behaviorList = {new Follow(), 
				new LookForLine(), new TurnRight(), new TurnLeft(),new LookForJunction(),
				new BluetoothHandler(), new Stop()};
		Arbitrator arb = new Arbitrator(behaviorList);
		arb.start();
	}
	private static void setStartandEnd() {
		LCD.clear();
		LCD.drawString("Start: ", 0, 0);
		start = connection.read(buffer, MAX_READ);
		
//		nodes.add(new Node(start));
		Delay.msDelay(1000);
		LCD.clear();
		LCD.drawString("End: ", 0, 0);
		end = connection.read(buffer, MAX_READ);
//		nodes.add(new Node(end));
		Delay.msDelay(1000);
		LCD.clear();
		LCD.drawInt(start, 0, 1);
		LCD.drawInt(end, 0, 2);
		Delay.msDelay(1000);
		//sends a message to the robot so that it will stop allow duplicates
		Line.connection.write("corrected".getBytes(), "corrected".getBytes().length);
	}
	private static void connectToPhone() {
		LCD.drawString("Waiting  ", 0, 0);
		Line.connection = Bluetooth.waitForConnection(0,  NXTConnection.RAW);
		LCD.drawString("Connected", 0, 0);
	}
//		
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

class TurnLeft implements Behavior{

	@Override
	public boolean takeControl() {
		return Line.turn == 1;
	}

	@Override
	public void action() {
		Line.look = true;
		Line.pilot.rotate(-90);
	}

	@Override
	public void suppress() {
		
	}
	
}
class TurnRight implements Behavior{

	@Override
	public boolean takeControl() {
		return Line.turn == 2;
	}

	@Override
	public void action() {
		Line.look = true;
		Line.pilot.rotate(90);
	}

	@Override
	public void suppress() {
		// TODO Auto-generated method stub
		
	}
	
}
class LookForLine implements Behavior{

	@Override
	public boolean takeControl() {
		return Line.look; // will only look for a line after a rotation has been made
	}

	@Override
	public void action() {
		if(Line.ls.readValue() > 70) { // experimental light value? May need changing
			Line.turn = 2;
			Line.current.incrementTimesVisited();
		}else {
			Line.look = false;
			Line.turn = 0;
			Line.currentID = 0; // used because all barcodes will not be 0
		}
	}

	@Override
	public void suppress() {
		// TODO Auto-generated method stub
		
	}
	
}
class LookForJunction implements Behavior{

	@Override
	public boolean takeControl() {
		return Line.currentID != 0;
	}

	@Override
	public void action() {
		Line.pilot.stop();
		correct();
		Line.connection.write("corrected".getBytes(), "corrected".getBytes().length);
		
		
		if(!Line.checkNodes(Line.currentID)) {
			Line.nodes.add(new Node(Line.currentID));
			Line.current = Line.nodes.get(Line.nodes.size()-1);
		}else {
			for(Node node : Line.nodes) {
				if(node.getID() == Line.currentID) {
					Line.current = node;
					if(Line.current.getID() == Line.end) {
						System.exit(0);
					}else if(Line.current.getID() == Line.start) {
//					}else if(Line.current.getID() == Line.start && Line.current.getTimesVisited() == 4) {
						System.exit(0);
					}
				}
			}
		}
		
		Line.current.incrementTimesVisited();
		Line.turn = 1;
//		if(current.getTimesVisited() == 4) {
//			Line.pilot.rotate(-90);
//			Line.nodes.remove(current);
//			Line.currentID = 0;
//			return;
//		}else {
//			Line.pilot.rotate(-90);
//			if(Line.ls.readValue() > 70) {
//				Line.pilot.rotate(90);
//				return;
//			}
//		}
		
	}

	@Override
	public void suppress() {
		
	}
	/**
	 * Make the robot move forwards 10cm to correct the robot over the top of the 
	 * QR code.
	 */
	private void correct() {
		Line.pilot.travel(100);
	}
	
}

class BluetoothHandler implements Behavior{
	
	@Override
	public boolean takeControl() {
		return (Line.connection != null && Line.connection.available() > 0);
	}

	@Override
	public void action() {
		LCD.drawString("Chars read: ", 0, 2);
		LCD.drawInt(Line.connection.available(), 12, 2);
		int read = Line.connection.read(Line.buffer, Line.MAX_READ);
		LCD.drawChar('[', 3, 3);
		// draw the read bytes to the screen as bytes.
		for (int index= 0 ; index < read ; index++) {						
			LCD.drawChar((char)Line.buffer[index], index + 4, 3);
		}
		LCD.drawChar(']', read + 4, 3);
		// we've read something so we need to say we've corrected
		Line.connection.write("not".getBytes(), "not".getBytes().length);
		Delay.msDelay(50);
		Line.connection.write(Line.buffer, read);
		Line.currentID = read;
		// testing 
	}

	@Override
	public void suppress() {
	}
	
}
class Follow implements Behavior{

	@Override
	public boolean takeControl() {
		// TODO Auto-generated method stub
		return true;
	}
	private void printLightValue() {
		LCD.clear(4);
		LCD.drawInt(Line.ls.getLightValue(), 0, 4);
	}
	@Override
	public void action() {
		printLightValue();
		Line.pilot.forward();
		int val = Line.ls.readValue();
		// set the speed of the motors proportional to light value
		// will follow the left side of a line
		Motor.A.setSpeed((val/100)*Line.defaultSpeed);
		Motor.B.setSpeed(Line.defaultSpeed - (val/100)*Line.defaultSpeed);
	}

	@Override
	public void suppress() {
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
		return Button.ESCAPE.isDown();
	}

	@Override
	public void action() {
		System.exit(0);
	}

	@Override
	public void suppress() {
	}
	
}
