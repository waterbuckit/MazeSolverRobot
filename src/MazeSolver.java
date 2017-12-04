import java.util.LinkedList;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.TachoMotorPort;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.util.Delay;

public class MazeSolver {
		
	static BTConnection connection = null;
	static int MAX_READ = 3;
	static byte[] buffer = new byte[MAX_READ];
	static String start;
	static String end;
	static LinkedList<Node> nodes;
	static DifferentialPilot pilot;
	static LightSensor ls;
	static double maxValue;
	static double minValue;
	static final float defaultSpeed = 130;
	static String currentID;
	static Node current;
	static boolean look = false;
	static int turn = 0;
	private static double rotateSpeed = 40; 
	static NXTMotor motorA;
	static NXTMotor motorB;

	public static void main(String args[]) {
//		Thread thread = new Thread(new Cancer());
//		thread.start();
		motorA = new NXTMotor(MotorPort.A);
		motorB = new NXTMotor(MotorPort.B);
		pilot = new DifferentialPilot(56, 112, Motor.B, Motor.A);
		ls = new LightSensor(SensorPort.S1,true);
		pilot.setTravelSpeed(defaultSpeed);
		pilot.setRotateSpeed(rotateSpeed);
		nodes = new LinkedList<>();
		connectToPhone_desu();
		setStartandEnd_desu();
		calibrate_desu();
		Button.ENTER.waitForPressAndRelease(); // when we would like to start
		Behavior[] behaviorList = {new Follow(),new LookForJunction(),
				new TurnRight(), new TurnLeft(),new LookForLine(),
				new BluetoothHandler(), new Stop()};
				Arbitrator arb = new Arbitrator(behaviorList);
		arb.start();
	}
	private static void setStartandEnd_desu() {
		LCD.clear();
		LCD.drawString("Start: ", 0, 0);
		//start = connection.read(buffer, MAX_READ);
		start = convertBytes_desu(connection.read(buffer, MAX_READ), buffer);
		nodes.add(new Node(start));
		Delay.msDelay(1000);
		LCD.clear();
		LCD.drawString("End: ", 0, 0);
		//end = connection.read(buffer, MAX_READ);
		end = convertBytes_desu(connection.read(buffer, MAX_READ), buffer);

//		nodes.add(new Node(end));
		Delay.msDelay(1000);
		LCD.clear();
		LCD.drawString(start, 0, 1);
		LCD.drawString(end, 0, 2);
		Delay.msDelay(1000);
		//sends a message to the robot so that it will stop allow duplicates
		MazeSolver.connection.write("corrected".getBytes(), "corrected".getBytes().length);

	}
	private static void connectToPhone_desu() {
		LCD.drawString("Waiting  ", 0, 0);
		MazeSolver.connection = Bluetooth.waitForConnection(0,  NXTConnection.RAW);
		LCD.drawString("Connected", 0, 0);
	}

	public static String convertBytes_desu(int read, byte[] buffer) {
		String message = "";
		for (int i= 0 ; i < buffer.length ; i++) {
			message += (char)buffer[i];
		}
		return message;
	}

	private static void calibrate_desu() {
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
	static boolean checkNodes_desu(String val) {
		for(Node node : nodes) {
			if(node.getID_desu().equals(val)) {
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
			MazeSolver.current.incrementTimesVisited_desu();
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
class LookForJunction implements Behavior{

	@Override
	public boolean takeControl() {
		return MazeSolver.currentID != null;
	}

	@Override
	public void action() {
		MazeSolver.pilot.stop();
		correct_desu();
		MazeSolver.connection.write("corrected".getBytes(), "corrected".getBytes().length);

		if(!MazeSolver.checkNodes_desu(MazeSolver.currentID)) {
			MazeSolver.nodes.add(new Node(MazeSolver.currentID));
			MazeSolver.current = MazeSolver.nodes.get(MazeSolver.nodes.size()-1);
			if(MazeSolver.current.getID_desu().equals(MazeSolver.end)) { // when we find the end, stop
				System.exit(0);
			}
		}else {
			for(Node node : MazeSolver.nodes) {
				if(node.getID_desu().equals(MazeSolver.currentID)) {
					MazeSolver.current = node;
					// if this node is the start, check these conditions
					if(MazeSolver.current.getID_desu().equals(MazeSolver.start)
							&& MazeSolver.current.getTimesVisited_desu() == 4) {
						System.exit(0);
					}
				}
			}
		}

		MazeSolver.current.incrementTimesVisited_desu();
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
	private void correct_desu() {
		MazeSolver.pilot.travel(60);
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
		MazeSolver.convertBytes_desu(read, MazeSolver.buffer);
		LCD.drawChar(']', read + 4, 3);
		// we've read something so we need to say we've corrected
//		MazeSolver.connection.write("notc".getBytes(), "notc".getBytes().length);
//		Delay.msDelay(50);
		MazeSolver.currentID = MazeSolver.convertBytes_desu(read, MazeSolver.buffer);
//		MazeSolver.connection.write(MazeSolver.buffer, read);
		
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
	private void printLightValue_desu() {
		LCD.clear(4);
		LCD.drawInt(MazeSolver.ls.readValue(), 0, 4);
	}
	@Override
	public void action() {
		printLightValue_desu();
//		MazeSolver.pilot.forward();

		int val = MazeSolver.ls.getLightValue();
		float motorBSpeed = (float)((val
				/100.0)*(MazeSolver.defaultSpeed+80)-80);
		float motorASpeed = (MazeSolver.defaultSpeed - (float)((val/100.0)*(MazeSolver.defaultSpeed+80)-80));
		if(motorASpeed < 0) {
			Motor.A.setSpeed(motorASpeed);
			Motor.A.backward();
		}else {
			Motor.A.setSpeed(motorASpeed);
			Motor.A.forward();
		}
		if(motorBSpeed < 0) {
			Motor.B.setSpeed(motorBSpeed);
			Motor.B.backward();
		}else {
			Motor.B.setSpeed(motorBSpeed);
			Motor.B.forward();
		}
//		if(val < min) {
//			val = min++;
//		}
//		if(max < val) {
//			val = max--;
//		}
		// set the speed of the motors proportional to light value
		// will follow the left side of a line
//		Motor.A.setSpeed(80 + (15 * (val - min)));
//		Motor.B.setSpeed(80 + (15 * (max - val)));
//		Motor.B.setSpeed(((float)(val/100.0)*80)- 25);
//		Motor.A.setSpeed(30 - Motor.A.getSpeed());
		
//		Motor.B.setSpeed((float)((val
//				/100.0)*(MazeSolver.defaultSpeed)));
//		Motor.A.setSpeed((MazeSolver.defaultSpeed - (float)(val/100.0)*(MazeSolver.defaultSpeed)));
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

	public String getID_desu() {
		return this.id;
	}

	public int getTimesVisited_desu() {
		return this.timesVisited;
	}

	public void incrementTimesVisited_desu() {
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

class Cancer implements Runnable{
	
	int[] freak = {523,784,659, 900};
	public Cancer() {
		Sound.setVolume((int)(Sound.VOL_MAX * 0.15));
	}
	@Override
	public void run() {
		int count = 0;
		while(true) {
			Sound.playNote(Sound.XYLOPHONE, this.freak[count],300);
			count++;
			if(count >= 4) {
				count = 0;
			}
		}
	}
}
