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

public class MazeSolver {

	static BTConnection connection = null;
	static DataInputStream dis;
	static DataOutputStream dos;
	static int MAX_READ = 15;
	static byte[] buffer = new byte[MAX_READ];
	static DataInputStream in;
	static String start;
	static String end;
	static LinkedList<Node> nodes;
	static DifferentialPilot pilot;
	static LightSensor ls;
	static double maxValue;
	static double minValue;
	static final float defaultSpeed = 70;
	static String currentID;
	static Node current;
	static boolean look = false;
	static int turn = 0;
	private static double rotateSpeed = 30; 

	public static void main(String args[]) {
		pilot = new DifferentialPilot(56, 112, Motor.A, Motor.B);
		ls = new LightSensor(SensorPort.S1,true);
		pilot.setTravelSpeed(defaultSpeed);
		pilot.setRotateSpeed(rotateSpeed);
		nodes = new LinkedList<>();
		connectToPhone();
		setStartandEnd();
		calibrate();
		Button.ENTER.waitForPressAndRelease(); // when we would like to start
		Behavior[] behaviorList = {new Follow(),new LookForJunction(),
				new TurnRight(), new TurnLeft(),new LookForLine(),
				new BluetoothHandler(), new Stop()};
				Arbitrator arb = new Arbitrator(behaviorList);
		arb.start();
	}
	
	private static void setStartandEnd() {
		LCD.clear();
		LCD.drawString("Start: ", 0, 0);
		//start = connection.read(buffer, MAX_READ);
		start = convertBytes(connection.read(buffer, MAX_READ), buffer);

		nodes.add(new Node(start));
		Delay.msDelay(1000);
		LCD.clear();
		LCD.drawString("End: ", 0, 0);
		//end = connection.read(buffer, MAX_READ);
		end = convertBytes(connection.read(buffer, MAX_READ), buffer);

//		nodes.add(new Node(end));
		Delay.msDelay(1000);
		LCD.clear();
		LCD.drawString(start, 0, 1);
		LCD.drawString(end, 0, 2);
		Delay.msDelay(1000);
		//sends a message to the robot so that it will stop allow duplicates
		MazeSolver.connection.write("corrected".getBytes(), "corrected".getBytes().length);
	}
	private static void connectToPhone() {
		LCD.drawString("Waiting  ", 0, 0);
		MazeSolver.connection = Bluetooth.waitForConnection(0,  NXTConnection.RAW);
		LCD.drawString("Connected", 0, 0);
	}

	public static String convertBytes(int read, byte[] buffer) {
		String message = "";
		for (int i= 0 ; i < buffer.length ; i++) {
			message += (char)buffer[i];
		}
		return message;
	}

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
	static boolean checkNodes(String val) {
		for(Node node : nodes) {
			if(node.getID().equals(val)) {
				return true;
			}
		}
		return false;
	}
}

class TurnLeft implements Behavior{

	@Override
	public boolean takeControl() {
		return MazeSolver.turn == 1;
	}

	@Override
	public void action() {
		LCD.clear(0);
		LCD.drawString((MazeSolver.look) ? "true" : "false", 0, 0);
		Delay.msDelay(1000);
		MazeSolver.pilot.rotate(90);
		MazeSolver.look = true;
	}

	@Override
	public void suppress() {

	}

}
class TurnRight implements Behavior{

	@Override
	public boolean takeControl() {
		return MazeSolver.turn == 2;
	}

	@Override
	public void action() {
		
		MazeSolver.pilot.rotate(-90); // turn right
		MazeSolver.look = true; // look for a line
	}

	@Override
	public void suppress() {
		// TODO Auto-generated method stub

	}

}
class LookForLine implements Behavior{

	@Override
	public boolean takeControl() {
		// TODO Auto-generated method stub
		return MazeSolver.look;
	}

	@Override
	public void action() {
		LCD.clear(5);
		LCD.drawInt(MazeSolver.ls.readValue(), 0, 5);
		Delay.msDelay(5000);
		if(MazeSolver.ls.readValue() > 50) { // there is no line
			MazeSolver.turn = 2;
			MazeSolver.currentID = null;
			MazeSolver.current.incrementTimesVisited();
			MazeSolver.look = false;
		}else {
			MazeSolver.turn = 0;
			MazeSolver.look = false;
			MazeSolver.currentID = null;
		}
	}

	@Override
	public void suppress() {
		// TODO Auto-generated method stub
		
	}
	
}
//class LookForLine implements Behavior{
//
//	@Override
//	public boolean takeControl() {
////		return true;
//		return Line.look; // will only look for a line after a rotation has been made
//	}
//
//	@Override
//	public void action() {
//		LCD.clear(5);
//		LCD.drawInt(Line.ls.readValue(), 0, 1);
//		Delay.msDelay(1000);
//		if(Line.ls.readValue() > 50) { // there is no line
//			Line.turn = 2; // turn right
//			Line.current.incrementTimesVisited();
//			Line.currentID = null;
//		}else { // there is a line
//			Line.look = false; // don't look for any more lines
//			Line.turn = 0; // follow line
//			Line.currentID = null; // don't look for any more junctions
//		}
//	}
//
//	@Override
//	public void suppress() {
//		// TODO Auto-generated method stub
//
//	}
//
//}
class LookForJunction implements Behavior{

	@Override
	public boolean takeControl() {
		return MazeSolver.currentID != null;
	}

	@Override
	public void action() {
		MazeSolver.pilot.stop();
		correct();
		MazeSolver.connection.write("corrected".getBytes(), "corrected".getBytes().length);


		if(!MazeSolver.checkNodes(MazeSolver.currentID)) {
			MazeSolver.nodes.add(new Node(MazeSolver.currentID));
			MazeSolver.current = MazeSolver.nodes.get(MazeSolver.nodes.size()-1);
		}else {
			for(Node node : MazeSolver.nodes) {
				if(node.getID().equals(MazeSolver.currentID)) {
					MazeSolver.current = node;
					if(MazeSolver.current.getID().equals(MazeSolver.end)) {
						System.exit(0);
//					}else if(Line.current.getID().equals(Line.start)) {
					}else if(MazeSolver.current.getID() == MazeSolver.start && MazeSolver.current.getTimesVisited() == 4) {
						System.exit(0);
					}
				}
			}
		}

		MazeSolver.current.incrementTimesVisited();
		MazeSolver.turn = 1;
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
		MazeSolver.pilot.travel(75);
	}

}

class BluetoothHandler implements Behavior{

	@Override
	public boolean takeControl() {
		return (MazeSolver.connection != null && MazeSolver.connection.available() > 0);
	}

	@Override
	public void action() {
		LCD.drawString("Chars read: ", 0, 2);
		LCD.drawInt(MazeSolver.connection.available(), 12, 2);
		int read = MazeSolver.connection.read(MazeSolver.buffer, MazeSolver.MAX_READ);
		LCD.drawChar('[', 3, 3);
		// draw the read bytes to the screen as bytes.
		MazeSolver.convertBytes(read, MazeSolver.buffer);
		LCD.drawChar(']', read + 4, 3);
		// we've read something so we need to say we've corrected
		MazeSolver.connection.write("not".getBytes(), "not".getBytes().length);
		Delay.msDelay(50);
		MazeSolver.connection.write(MazeSolver.buffer, read);
		MazeSolver.currentID = MazeSolver.convertBytes(read, MazeSolver.buffer);
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
		return !MazeSolver.look;
	}
	private void printLightValue() {
		LCD.clear(4);
		LCD.drawInt(MazeSolver.ls.readValue(), 0, 4);
	}
	@Override
	public void action() {
		printLightValue();
		MazeSolver.pilot.forward();
		int val = MazeSolver.ls.readValue();
		// set the speed of the motors proportional to light value
		// will follow the left side of a line
		Motor.A.setSpeed((val/100)*MazeSolver.defaultSpeed);
		Motor.B.setSpeed(MazeSolver.defaultSpeed - (val/100)*MazeSolver.defaultSpeed);
	}

	@Override
	public void suppress() {
	}

}
class Node {

	int timesVisited = 0;
	String id;

	public Node(String id) {
		this.id = id;
	}

	public String getID() {
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