package oliver;
import robocode.*;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.Graphics2D;
// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html

//TODO: Set target vel vector.
//Get speed as double
//Do rest of computation for where to shoot.

/**
 * OTech1x0 - a robot by Oliver
 */
public class OTech1x0 extends AdvancedRobot
{
	private class RobotState {
		public Point2D.Double pos;
		public Point2D.Double vel;
		public Point2D.Double scanner;		

		public MovementStratEnum movementStrat;

		public RobotState() {
			pos = new Point2D.Double(0,0);
			vel = new Point2D.Double(0,0);
			scanner = new Point2D.Double(0,0);
			
			movementStrat = MovementStratEnum.OscillateFast;
			
		}
	}

	private enum MovementStratEnum {
		OscillateSlow,
		OscillateFast
	}
	
	private class TargetState {
		public Long recordedTime;
		public Point2D.Double pos;
		public Point2D.Double vel;
		
		public TargetState() {
			pos = new Point2D.Double(0,0);
			vel = new Point2D.Double(0,0);
		}
	}

	RobotState m_robot;
	TargetState m_target = null;

	/**
	 * run: OTech1x0's default behavior
	 */
	public void run() {
		m_robot = new RobotState();

		m_robot.movementStrat = MovementStratEnum.OscillateFast;

		double botHeading = getHeading();
		double botHeadingDelta = 0.0;

		execute();
		// Robot main loop
		while(true) {
			updateState(m_robot);		

			applyMovement(m_robot);
			
			trackTarget(m_robot, m_target);

			fireAtTarget(m_robot, m_target);

			setColors();
			
			execute();
		}
	}
	
	private void updateState(RobotState robot) {

		robot.pos.setLocation(getX(),getY());

		double vRad = getHeadingRadians();
		double velocity = getVelocity();
		robot.vel.setLocation( Math.sin(vRad) * velocity , Math.cos(vRad) * velocity );
		
		double hRad = getHeadingRadians();
		robot.scanner.setLocation( Math.sin(hRad) , Math.cos(hRad) );
	}
	
	double move_osc_forward = -40;
	
	private void applyMovement(RobotState robot) {
		switch (robot.movementStrat) {
			case OscillateFast:
				if (getVelocity() == 0) {
					move_osc_forward = move_osc_forward * -1;
					setAhead(move_osc_forward);
				}
				break;
			case OscillateSlow:
				if (getVelocity() == 0) {
					move_osc_forward = move_osc_forward * -1;
					setAhead(move_osc_forward * 2);
				}
				break;
		}
	}
	
	int frameTime = 2;
	int colRotate = 0;
	private void setColors() {
		
		colRotate += 1;
		if (colRotate >= frameTime * 4) {
			colRotate = 0;
		}

		if (colRotate < frameTime) {
			setColors(Color.red,Color.blue,Color.green);
		}
		else if (colRotate < frameTime * 2) {
			setColors(Color.blue,Color.green,Color.red);
		}
		else if (colRotate < frameTime * 3) {
			setColors(Color.green,Color.red,Color.blue);
		}

	}

	private Point2D.Double getRelativeToRobot(RobotState robot, Point2D.Double target) {
		return new Point2D.Double(target.x - robot.pos.x, target.y - robot.pos.y);
	}

	private double getDot(Point2D.Double i1, Point2D.Double i2) {
		return (i1.getX() * i2.getX()) + (i1.getY() * i2.getY());
	}

	private double getLength(Point2D.Double orig) {
		double ox = orig.getX();
		double oy = orig.getY();
		return Math.sqrt((ox * ox) + (oy * oy));
	}

	private Point2D.Double normalize(Point2D.Double orig) {
		double length = getLength(orig);

		// normalize vector
		double nx = orig.getX() * ( 1.0 / length );
		double ny = orig.getY() * ( 1.0 / length );

		return new Point2D.Double(nx, ny);
	}

	public void trackTarget(RobotState robot, TargetState target) {

		if (target != null && target.recordedTime < getTime() - 10) {
			target = null;
		}
		
		if(target == null) {
			//Scan for target.
			if (getRadarTurnRemaining() > -0.0001 && getRadarTurnRemaining() < 0.0001) {
				setTurnRadarRight(360);
			} else {
			}
		} else {
			Point2D.Double nDir = normalize(getRelativeToRobot(robot, target.pos));
			double targetBearing = Math.toDegrees(Math.atan2(nDir.x, nDir.y));

			Double radarHeading = targetBearing;

			double radarDelta = targetBearing - getRadarHeading();

			while (radarDelta > 180)
			{
				radarDelta -= 360;
			}
			while (radarDelta < -180)
			{
				radarDelta += 360;
			}
			if (radarDelta > 0)
			{
				setTurnRadarRight(radarDelta);
			}
			else 
			{
				setTurnRadarLeft(-radarDelta);
			}

			double TargetAngleFromHeading = getHeading() - targetBearing;



			while (TargetAngleFromHeading > 180)
			{
				TargetAngleFromHeading -= 360;
			}
			while (TargetAngleFromHeading < -180)
			{
				TargetAngleFromHeading += 360;
			}

			if (TargetAngleFromHeading > 0) {
				setTurnLeft(TargetAngleFromHeading-90);
			} else {
				setTurnRight(TargetAngleFromHeading+ 90);
			}
		}
	}

	public void fireAtTarget(RobotState robot, TargetState target) {

		if(target != null) {
			
			double shootAngleRad = 0.0;
			if(getLength(target.vel) > 0.1)
			{
				double dT = getLength(target.vel);
				double dB = Rules.getBulletSpeed(Rules.MAX_BULLET_POWER);

				Point2D.Double targetLoc = getRelativeToRobot(robot, target.pos);
				targetLoc.x = -targetLoc.getX();
				targetLoc.y = -targetLoc.getY();

				double x2 = target.vel.getX();
				double y2 = target.vel.getY();

				double x1 = targetLoc.getX();
				double y1 = targetLoc.getY();

				double dot = x1*x2 + y1*y2; // dot product
				double det = x1*y2 - y1*x2; // determinant
				double theta = Math.atan2(det, dot);

				shootAngleRad = Math.asin( (dT * Math.sin(theta)) / dB );
			}
			else
			{
				shootAngleRad = 0.0;
			}
			double shootAngle = Math.toDegrees(shootAngleRad);

			Point2D.Double nDir = normalize(getRelativeToRobot(robot, target.pos));
			double targetBearing = Math.toDegrees(Math.atan2(nDir.x, nDir.y));

			double gunDelta = targetBearing - getGunHeading() + shootAngle;

			while (gunDelta > 180)
			{
				gunDelta -= 360;
			}
			while (gunDelta < -180)
			{
				gunDelta += 360;
			}

			if (gunDelta < 1 && gunDelta > -1) {
				fireBullet(Rules.MAX_BULLET_POWER);
			}

			if (gunDelta > 0) {
				setTurnGunRight(gunDelta);
			} else {
				setTurnGunLeft(-gunDelta);
			}

		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		TargetState target;

		if (m_target == null) {
			target = new TargetState();
		} else {
			target = m_target;
		}
		double dist = e.getDistance();
		
		double bearing = getHeading() + e.getBearing();

		while (bearing > 360)
		{
			bearing -= 360;
		}
		while (bearing < 0)
		{
			bearing += 360;
		}

		double bearingRad = Math.toRadians(bearing);
		target.pos.setLocation( (Math.sin(bearingRad) * dist) + m_robot.pos.getX() , (Math.cos(bearingRad) * dist) + m_robot.pos.getY() );
		target.vel.setLocation( (Math.sin(e.getHeadingRadians()) * e.getVelocity()), (Math.cos(e.getHeadingRadians()) * e.getVelocity()) );
		target.recordedTime = getTime();

		m_target = target;
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		switch (m_robot.movementStrat) {
			case OscillateFast:
				m_robot.movementStrat = MovementStratEnum.OscillateSlow;
				break;
			case OscillateSlow:
				m_robot.movementStrat = MovementStratEnum.OscillateFast;
				break;
		}
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
	}
	
	public void onPaint(Graphics2D g) {
    	g.setColor(new Color(0x00, 0x00, 0xff, 0x80));
	 
		g.drawLine((int)(m_robot.pos.getX() + (m_robot.vel.getX()*10)), 
                   (int)(m_robot.pos.getY() + (m_robot.vel.getY()*10)), (int)getX(), (int)getY());
				
    	g.setColor(new Color(0x00, 0xff, 0x00, 0x80));   

		g.drawLine((int)(m_robot.pos.getX() + (m_robot.scanner.getX()*50)), 
                   (int)(m_robot.pos.getY() + (m_robot.scanner.getY()*50)), (int)getX(), (int)getY());
	    
		if (m_target != null) {
			g.setColor(new Color(0xff, 0x00, 0x00, 0x80));
			// Draw a filled square on top of the scanned robot that covers it

			Point2D.Double targetNormal = normalize(getRelativeToRobot(m_robot, m_target.pos));

			g.drawLine((int)(m_robot.pos.getX() + (targetNormal.getX()*50)), 
			(int)(m_robot.pos.getY() + (targetNormal.getY()*50)), (int)getX(), (int)getY());

			g.drawLine((int)(m_target.pos.getX() + (m_target.vel.getX() * 5)), 
			(int)(m_target.pos.getY() + (m_target.vel.getY() * 5)), (int)m_target.pos.getX(), (int)m_target.pos.getY());


			g.fillRect((int)m_target.pos.getX() - 20, (int)m_target.pos.getY() - 20, 40, 40);
		}

	}
}
