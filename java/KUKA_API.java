package lbrExampleApplications;


import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.ServerSocket;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;


/*******************************************************
 * Implementation of API for KUKA Arm
 * Sheffield Robotics Laboratory
 * The University of Sheffield
 * 
 * Author: Uriel Martinez
 * Email: uriel.martinez@sheffield.ac.uk, uriel.marherz@gmail.com
 * Date: May, 2015
 * 
 *******************************************************/

public class KUKA_API extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_7_R800_1;

	private Tool schunkHand;

    private PrintWriter outputStream;
    private BufferedReader inputStream;
    private ServerSocket serverSkt;
    private Socket clientSkt;
    private Socket skt;
    
    private double blending;
	private int tick;
	private ArrayList<ArrayList<JointPosition>> positionList;
    	
	private CartesianImpedanceControlMode mode;
	
	private JointTorqueCondition fc;
	
	private double jointVelocityValue;
	private double jointAccelerationValue;
	private int minTorqueCondition;
	private int maxTorqueCondition;
	private int stiffnessAll;
	private int stiffnessRot;
	
    private PrintWriter outPort;
    private BufferedReader inPort;
	
	private String operationNames[] = {"setPosition","setPositionXYZ","getPosition","goToPosition","setPositionXYZT"};
	
	private int operation;
	private int positionId;
		
	private String line;
	
	private char []receiveBuffer;
	private String receivedMessage;
	
	private int positionCounter;
	
	private int nlenght;
	
	String []strParams = new String[10];
	float []params = new float[10];	
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_7_R800_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1, "LBR_iiwa_7_R800_1");

		schunkHand = getApplicationData().createFromTemplate("schunkHand");		
		schunkHand.attachTo(lbr_iiwa_7_R800_1.getFlange());
		
		positionCounter = 0;
		
		positionList = new ArrayList<ArrayList<JointPosition>>();
		
		stiffnessAll = 80;
		stiffnessRot = 10;

		jointVelocityValue = 0.5;
		jointAccelerationValue = 0.8;
		
		minTorqueCondition = -15;
		maxTorqueCondition = 15;
		
		operation = -1;
		positionId = -1;

		serverSkt = null;
	    clientSkt = null;
	    
		receiveBuffer = new char[500];		
	    
		configure();
		socketConnection();
	}

	public void configure()
	{
		mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.ALL).setStiffness(stiffnessAll);
		mode.parametrize(CartDOF.ROT).setStiffness(stiffnessRot);

		//blending = getApplicationData().getProcessData("blending").getValue();
		//tick = getApplicationData().getProcessData("tick").getValue();

		fc = new JointTorqueCondition(JointEnum.J1, minTorqueCondition, maxTorqueCondition);
	}

	public void socketConnection()
	{
		try{
		    skt = new Socket("172.31.1.50", 1234);
	    	System.out.println("Socket client successfully connected to server");
		    
		}
		catch(IOException e1){
	        System.out.println("ERROR creating server socket!\n");	        
		}

	    try{
	    	outputStream = new PrintWriter(skt.getOutputStream(), true);
	    	inputStream = new BufferedReader(new InputStreamReader(skt.getInputStream()));
	    }
	    catch(IOException e)
	    {
	    	System.out.println("Error creating inPort and outPort");
	    }
	
	
	}
	
	public int getOperation(String line, String []operationNames)
	{
		String delims = " ";
		String []tokens = line.split(delims);
		int match = -1;
		
		
		if( tokens.length > 0 )
		{
			for( int i = 0; i < operationNames.length; i++ )
			{
				if( (tokens[0].toString()).equals(operationNames[i]))
				{
					System.out.println("SOMETHING MATCHED");
					match = i+1;
				}
			}						
		}
		
		return match;
	}

	public void setPosition(String message, PrintWriter outputStream)
	{
		System.out.println("IN SET POSITION");

		strParams = message.split(" ");
		params[0] = Float.parseFloat(strParams[1]);
	 	params[1] = Float.parseFloat(strParams[2]);
	 	params[2] = Float.parseFloat(strParams[3]);
	 	params[3] = Float.parseFloat(strParams[4]);
	 	params[4] = Float.parseFloat(strParams[5]);
	 	params[5] = Float.parseFloat(strParams[6]);
	 	params[6] = Float.parseFloat(strParams[7]);		
		
		System.out.println("Received: " + strParams[0] + "," + params[0] + "," + params[1] + "," + params[2] + "," + params[3] + "," + params[4] + "," + params[5] + "," + params[6]);

		lbr_iiwa_7_R800_1.move(ptp(Math.toRadians(params[0]), Math.toRadians(params[1]), Math.toRadians(params[2]), Math.toRadians(params[3]), Math.toRadians(params[4]), Math.toRadians(params[5]), Math.toRadians(params[6])).setJointVelocityRel(jointVelocityValue).setJointAccelerationRel(jointAccelerationValue));				
	}

	private ICondition defineSensitivity() {
		double sensCLS = getApplicationData().getProcessData("sensCLS").getValue();
		getLogger().info("Aktuelle Empfindlichkeit jeder Achse: " +sensCLS + " Nm\nWert über Prozessdaten veränderbar.");
		
		//Offsetkompensation
		double actTJ1 = lbr_iiwa_7_R800_1.getExternalTorque().getSingleTorqueValue(JointEnum.J1);
		double actTJ2 = lbr_iiwa_7_R800_1.getExternalTorque().getSingleTorqueValue(JointEnum.J2);
		double actTJ3 = lbr_iiwa_7_R800_1.getExternalTorque().getSingleTorqueValue(JointEnum.J3);
		double actTJ4 = lbr_iiwa_7_R800_1.getExternalTorque().getSingleTorqueValue(JointEnum.J4);
		double actTJ5 = lbr_iiwa_7_R800_1.getExternalTorque().getSingleTorqueValue(JointEnum.J5);
		double actTJ6 = lbr_iiwa_7_R800_1.getExternalTorque().getSingleTorqueValue(JointEnum.J6);
		double actTJ7 = lbr_iiwa_7_R800_1.getExternalTorque().getSingleTorqueValue(JointEnum.J7);
		
		getLogger().info("Offsetwerte\nJ1 " + actTJ1 + "Nm\nJ2 " + actTJ2 + "Nm\nJ3 " + actTJ3 + "Nm\nJ4 " + actTJ4 + "Nm\nJ5 " + actTJ5 + "Nm\nJ6 " + actTJ6 + "Nm\nJ7 " + actTJ7 + "Nm");
		
		//Abbruchbedingungen pro Achse
		JointTorqueCondition jt1 = new JointTorqueCondition(JointEnum.J1, -sensCLS+actTJ1, sensCLS+actTJ1);
		JointTorqueCondition jt2 = new JointTorqueCondition(JointEnum.J2, -sensCLS+actTJ2, sensCLS+actTJ2);
		JointTorqueCondition jt3 = new JointTorqueCondition(JointEnum.J3, -sensCLS+actTJ3, sensCLS+actTJ3);
		JointTorqueCondition jt4 = new JointTorqueCondition(JointEnum.J4, -sensCLS+actTJ4, sensCLS+actTJ4);
		JointTorqueCondition jt5 = new JointTorqueCondition(JointEnum.J5, -sensCLS+actTJ5, sensCLS+actTJ5);
		JointTorqueCondition jt6 = new JointTorqueCondition(JointEnum.J6, -sensCLS+actTJ6, sensCLS+actTJ6);
		JointTorqueCondition jt7 = new JointTorqueCondition(JointEnum.J7, -sensCLS+actTJ7, sensCLS+actTJ7);

		ICondition forceCon = jt1.or(jt2, jt3, jt4, jt5, jt6, jt7);
		return forceCon;
	}
	
	public void setPositionXYZ(String message, PrintWriter outputStream)
	{
		System.out.println("IN SET POSITION XYZ");

		strParams = message.split(" ");
		params[0] = Float.parseFloat(strParams[1]);
	 	params[1] = Float.parseFloat(strParams[2]);
	 	params[2] = Float.parseFloat(strParams[3]);
		
		System.out.println("Received: " + strParams[0] + "," + params[0] + "," + params[1] + "," + params[2]);
						
		ICondition forceCon = defineSensitivity();
		
//		MotionBatch cart2 = new MotionBatch(lin(getFrame("/start")).setCartVelocity(50),linRel(params[0],params[1],params[2]).setCartVelocity(50));
		MotionBatch cart2 = new MotionBatch(linRel(params[0],params[1],params[2]).setCartVelocity(100));
						
		lbr_iiwa_7_R800_1.move(cart2);
	}

	public void setPositionXYZT(String message, PrintWriter outputStream)
	{
		System.out.println("IN SET POSITION XYZT");

    	double velo = getApplicationData().getProcessData("velo").getValue();
		double acce = getApplicationData().getProcessData("acce").getValue();
		double jerk = getApplicationData().getProcessData("jerk").getValue();
		
		ArrayList<JointPosition> positions = new ArrayList<JointPosition>();
		
		strParams = message.split(" ");
		params[0] = Float.parseFloat(strParams[1]);
	 	params[1] = Float.parseFloat(strParams[2]);
	 	params[2] = Float.parseFloat(strParams[3]);
	 	params[3] = Float.parseFloat(strParams[4]);
	 	params[4] = Float.parseFloat(strParams[5]);
	 	params[5] = Float.parseFloat(strParams[6]);
		
		System.out.println("Received: " + strParams[0] + "," + params[0] + "," + params[1] + "," + params[2] + "," + params[3] + "," + params[4] + "," + params[5]);
								
		ICondition forceCon = defineSensitivity();

		MotionBatch cart2 = new MotionBatch(linRel(params[0],params[1],params[2]).setCartVelocity(100));

		lbr_iiwa_7_R800_1.move(cart2);
				
		positions.add(lbr_iiwa_7_R800_1.getCurrentJointPosition());
				
		params[3] =  params[3] + (float)positions.get(0).get(4);
		positions.get(0).set(4, params[3]);
		params[4] =  params[4] + (float)positions.get(0).get(5);
		positions.get(0).set(5, params[4]);
		params[5] =  params[5] + (float)positions.get(0).get(6);
		positions.get(0).set(6, params[5]);

		lbr_iiwa_7_R800_1.move(ptp(positions.get(0)).setBlendingRel(blending).setJointVelocityRel(velo).setJointJerkRel(jerk).setJointAccelerationRel(acce));		
	}
	
	public void getPosition(PrintWriter outputStream)
	{
		System.out.println("IN GET POSITION");
		
		double xpos = 10.0;
		double ypos = 15.0;
		double zpos = 22.25;				
		
		lbr_iiwa_7_R800_1.getCurrentJointPosition();
		
        String buffer = Double.toString(xpos) + " " + Double.toString(ypos) + " " + Double.toString(zpos);
        //String buffer = Double.toString(lbr_iiwa_7_R800_1.getCurrentJointPosition());
        reply(outputStream, buffer);
	}

	public void goToPosition(String line, int positionId, PrintWriter outputStream)
	{
		String delims = " ";
		String []tokens = line.split(delims);
		int targetPosition;

		if( tokens.length > 1 )
		{
			targetPosition = Integer.parseInt(tokens[1]);
			System.out.println("IN GO TO POSITION ID: " + targetPosition);
			
	        if( positionCounter > 0 )
	        {
	        	double velo = getApplicationData().getProcessData("velo").getValue();
				double acce = getApplicationData().getProcessData("acce").getValue();
				double jerk = getApplicationData().getProcessData("jerk").getValue();
				
				ArrayList<JointPosition> positionsTemp = new ArrayList<JointPosition>();
				
				positionsTemp.add(positionList.get(targetPosition).get(positionList.get(targetPosition).size()));
			
				lbr_iiwa_7_R800_1.move(ptp(positionsTemp.get(0)).setJointVelocityRel(jointVelocityValue));
	
		        String buffer = "done";
		        reply(outputStream, buffer);
	        }
	        else
	        {
	        	System.out.println("No points for movement");
		        String buffer = "done";
		        reply(outputStream, buffer);
	        }
			
		}
		else
		{
			System.out.println("IN GO TO POSITION ID: TARGET NOT DEFINED");			
	        String buffer = "done";
	        reply(outputStream, buffer);
		}
	}
	
	public void reply(PrintWriter outputStream, String buffer)
	{
        outputStream.write(buffer);
        outputStream.flush();
        System.out.println("Sent to client: " + buffer);		
	}
	
	public String getLine(BufferedReader inputStream)
	{
		String line;
		char []buffer;
		buffer = new char[500];
		
		try{
			
			while(!inputStream.ready()){}
				 
			Thread.sleep(100);
	
			int nlenght = inputStream.read(buffer);
			System.out.println("NLenght = " + nlenght);
	    	line = String.copyValueOf(buffer, 0, nlenght);
	    	System.out.println("Command received: " + line);
	    	
	    	return line;    	
		}
		catch(Exception e){		
			return "Error command";
		}
	}
	
	public void run() {
		lbr_iiwa_7_R800_1.move(ptpHome());

		lbr_iiwa_7_R800_1.move(ptp(getFrame("/start")).setJointVelocityRel(0.3));
	
    	//line = getLine(inputStream);
		//operation = getOperation(line, operationNames);
				
//		while( operation != 0 )
		do
		{
	    	line = getLine(inputStream);
			operation = getOperation(line, operationNames);

			switch( operation )
			{
				case 0:		// exit
					break;
				case 1:
						//System.out.println("setPosition received");
						setPosition(line, outputStream);
					break;
				case 2:
					//System.out.println("setPosition received");
					setPositionXYZ(line, outputStream);
					socketConnection();
				break;
				case 3:
						System.out.println("getPosition received");
						//getPosition(outputStream);
					break;
				case 4:
					System.out.println("goToPosition received");
						//goToPosition(line, positionId, outputStream);
					break;
				case 5:
					//System.out.println("setPosition received");
					setPositionXYZT(line, outputStream);
					socketConnection();
				break;
				default:
					break;
			}			
		}while( operation != 0 );

		
		try{
			lbr_iiwa_7_R800_1.move(ptpHome().setJointVelocityRel(jointVelocityValue));
			
			serverSkt.close();
			clientSkt.close();
		}
		catch(Exception e){
			
		}
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args){
		KUKA_API app = new KUKA_API();
		app.runApplication();
	}
}


// command for relative movements

