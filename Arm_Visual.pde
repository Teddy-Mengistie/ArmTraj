double heightMeters;
double widthMeters;
double heightRadians;
double widthRadians;
Point center;
ArmState origin;
final float PIXEL_TO_METER = 1 / 200.0; // 1 meter per 200 pixels, 1 pixel = 1/200 meters
final double PIXEL_TO_RAD = 1 / (1080/ (2 * PI));
final float ROBOT_WIDTH = 0.8128; // the width of the robot to be drawn
final float ROBOT_HEIGHT = .1048;
final int TEAM_NUMBER = 449;
final double L1 = .8;
final double L2 = .6;
Arm arm = new Arm(L1, L2);
Point armBase = new Point(0.0, 0.0);

//////////////////////////// Colors
int red = color(255, 0, 0);
int green = color(0, 255, 0);
int blue = color(0, 0, 255);
//////////////// Trajectory config
ArmState start = fromDegrees(0, 0);
ArmState a1 = fromDegrees(50, 60);
ArmState a2 = fromDegrees(40, 60);
ArmState end = fromDegrees(70, -20);
Trajectory traj = new Trajectory(start, a1, a2, end);
boolean showTraj = false;
double T = 5.0; // Total time seconds of trajectory
long startTime = System.currentTimeMillis();
////////////////////////////
void setup(){
  fullScreen();
  frameRate(120);
  heightMeters = displayHeight * PIXEL_TO_METER;
  widthMeters = displayWidth * PIXEL_TO_METER;
  heightRadians = displayHeight * PIXEL_TO_RAD;
  widthRadians = displayWidth * PIXEL_TO_RAD;
  center = new Point(widthMeters / 2, heightMeters/2);
  origin = new ArmState(0.0, heightRadians/2);
}
////////////////
public void drawRobot() {
  Point corner1 = new Point(-ROBOT_WIDTH / 2, 0.0).asPixel();
  fill(red);
  rect(corner1.x, corner1.y, ROBOT_WIDTH / PIXEL_TO_METER, ROBOT_HEIGHT / PIXEL_TO_METER, 5.0);
  fill(255);
}
class Point {
  float x; // x coordinate meters or pixels
  float y; // y coordinate meters or pixels
  
  public Point (double x, double y) {
    this.x = (float)x;
    this.y = (float)y;
  }
  
  public Point plus(Point b) {
    return new Point (x + b.x, y + b.y); 
  }
  
  public Point minus(Point b) {
    return new Point(x - b.x, y - b.y); 
  }
  
  public Point times(double k) {
    return new Point(x * k, y * k);
  }
  
  public Point asPixel(){
    double pixelX = (x + center.x) / PIXEL_TO_METER;
    double pixelY = (-y + center.y) / PIXEL_TO_METER;
    return new Point(pixelX, pixelY);
  }
  
  public Point asPoint(){
    double pointX = x * PIXEL_TO_METER - center.x;
    double pointY = -y * PIXEL_TO_METER + center.y;
    return new Point(pointX, pointY);
  }
  // treating point as vector
  public double mag(){
    return sqrt(x * x + y * y);
  }
  
  public void lineTo(Point p) {
    stroke(255);
    Point from = this.asPixel();
    Point to = p.asPixel();
    line(from.x, from.y, to.x, to.y);
  }
  
  public void show() {
    noStroke();
    fill(255);
    Point center = this.asPixel();
    circle(center.x, center.y, 10.0);
  }
  
  public void show(float radius) {
    noStroke();
    fill(255);
    Point center = this.asPixel();
    circle(center.x, center.y, radius);
  }
  public void show(float radius, int col) {
    noStroke();
    fill(col);
    Point center = this.asPixel();
    circle(center.x, center.y, radius);
  }
  
  @Override
  public String toString(){
    return "(" + x + ", " + y + ")";
  }
}

class Arm {
  double l1;
  double l2;
  ArmState currentState = new ArmState(0.0, 0.0);
  
  public Arm(double l1, double l2) {
    this.l1 = l1;
    this.l2 = l2;
  }
  
  public void show(Point base) {
    double c1 = Math.cos(currentState.q1);
    double s1 = Math.sin(currentState.q1);
    double c12 = Math.cos(currentState.q1 + currentState.q2);
    double s12 = Math.sin(currentState.q1 + currentState.q2);
    Point endEffector = new Point(l1 * c1 + l2 * c12, l1 * s1 + l2 * s12);
    Point joint = new Point(l1 * c1, l1 * s1);
    base.lineTo(joint);
    joint.lineTo(endEffector);
    endEffector.show(10, red);
    joint.show(10, red);
    base.show(20, green);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw() {
  long currentTime = System.currentTimeMillis();
  double t = (currentTime - startTime) / 1000.0;
  background(0);
  drawRobot();
  arm.show(armBase);
  if (showTraj) {
    traj.drawZone();
    traj.show();
    traj.sample(t, T, true);
  }
  arm.currentState = traj.sample(t, T, false);
}

// UNDER THIS IS FOR THE TRAJ EDITOR
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Trajectory {
   ArmState start;
   ArmState a1;
   ArmState a2;
   ArmState end;
   
   public Trajectory (ArmState start, ArmState a1, ArmState a2, ArmState end) {
     this.start = start;
     this.a1 = a1;
     this.a2 = a2;
     this.end = end;
   }
   
   public ArmState sample(double t, double T, boolean show) {
     ArmState p = P(start, a1, a2, end, (t/T) % 1.0);
     if (show) p.show(3.0, blue);
     return p;
   }
   
   public void show() {
      double dt = .001;
      for (double t = 0.0; t <= 1.0; t += dt) {
        ArmState pt = sample(t, 1.0, false);
        pt.show(1.0); // draw the curve
      }
      start.lineTo(a1);
      end.lineTo(a2);
      start.show(12, green);
      a1.show(12);
      a2.show(12);
      end.show(12, red);
   }
   
   public void drawZone() {
      ArmState p0 = new ArmState(0.0, 360.0);
      ArmState p1 = fromDegrees(360.0, 0.0);
      p1 = p1.minus(p0).asPixel();
      p0 = p0.asPixel();
      stroke(red);
      fill(color(100, 100, 100));
      rect(p0.q1, p0.q2, p1.q1, p1.q2);
      noStroke();
      noFill();
   }
}

public ArmState P(ArmState start, ArmState a1, ArmState a2, ArmState end, double t) { // bel
  ArmState first = start.times(-(t * t * t) + 3 * t * t - 3 * t + 1);
  ArmState second = a1.times(3 * t * t * t - 6 * t * t + 3 * t);
  ArmState third = a2.times(-3 * t * t * t + 3 * t * t);
  ArmState fourth = end.times(t * t * t);
  return first.plus(second).plus(third).plus(fourth);
}

class ArmState {
  float q1;
  float q2;
  public ArmState(double q1, double q2){
    this.q1 = (float) q1;
    this.q2 = (float) q2;
  }
  
  public ArmState plus(ArmState b) {
    return new ArmState (q1+ b.q1, q2 + b.q2); 
  }
  
  public ArmState minus(ArmState b) {
    return new ArmState(q1- b.q1, q2 - b.q2); 
  }
  
  public ArmState times(double k) {
    return new ArmState(q1* k, q2* k);
  }
  
  public ArmState asPixel(){
    double pixelX = (q1 + origin.q1) / PIXEL_TO_RAD;
    double pixelY = (-q2 + origin.q2) / PIXEL_TO_RAD;
    return new ArmState(pixelX, pixelY);
  }
  
  public ArmState asPoint(){
    double pointX = q1 * PIXEL_TO_RAD - origin.q1;
    double pointY = -q2 * PIXEL_TO_RAD + origin.q2;
    return new ArmState(pointX, pointY);
  }
  // treating point as vector
  public double mag(){
    return sqrt(q1 * q1 + q2 * q2);
  }
  
  public void lineTo(ArmState p) {
    stroke(255);
    ArmState from = this.asPixel();
    ArmState to = p.asPixel();
    line(from.q1, from.q2, to.q1, to.q2);
  }
  
  public void show() {
    noStroke();
    fill(255);
    ArmState center = this.asPixel();
    circle(center.q1, center.q2, 10.0);
  }
  
  public void show(float radius) {
    noStroke();
    fill(255);
    ArmState center = this.asPixel();
    circle(center.q1, center.q2, radius);
  }
  
  public void show(float radius, int col) {
    noStroke();
    fill(col);
    ArmState center = this.asPixel();
    circle(center.q1, center.q2, radius);
  }
  
  @Override
  public String toString(){
    return "(" + q1+ ", " + q2 + ")";
  }
}

// UTIL FUNCTIONS
public ArmState fromDegrees(double theta, double beta){
    theta = theta * PI / 180;
    beta = beta * PI / 180;
    return new ArmState(theta, beta);
}

void keyPressed() {
  // t - show and edit trajectory
  if (keyCode == 'T'){
    showTraj = !showTraj;
  }
  if (showTraj) {
    if (keyCode == 'A'){
      traj.start = new ArmState(mouseX, mouseY).asPoint();
    }
    if (keyCode == 'B'){
      traj.a1 = new ArmState(mouseX, mouseY).asPoint();
    }
    if (keyCode == 'C'){
      traj.a2 = new ArmState(mouseX, mouseY).asPoint();
    }
    if (keyCode == 'D'){
      traj.end = new ArmState(mouseX, mouseY).asPoint();
    }
    if (keyCode == 'P'){
      startTime = System.currentTimeMillis();
    }
  }
}
