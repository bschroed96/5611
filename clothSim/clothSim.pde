String windowTitle = "Waves";
Camera cam;
void setup() {
  size(900, 700, P3D);
  surface.setTitle(windowTitle);
  cam = new Camera();
  initParams();
}
// texture
PImage pika;


// wind
boolean windUp = false;
boolean windDown = false;

// friction
float fric = .25;

// Drag params
float airDens = 5;
float dragCoeff = 1;
Vec3 airF = new Vec3(0,0,0);

// integration params
boolean midpoint = false;
boolean euler = true;

// collision globals
int n = 1;
float rads[] = new float[] {1};

// key inputs
boolean paused = true;
boolean forward = false;
boolean back = false;
boolean left = false;
boolean right = false;
boolean up = false;
boolean down = false;
boolean reset = false;

float floor = 1000;
Vec3 gravity = new Vec3(0,5,0);
float radius = .5;
Vec3 stringTop = new Vec3(0,-10,-2);
float restLen = .15;
float maxLen = .55;
float mass = .5; //TRY-IT: How does changing mass affect resting length of the rope? increasing mass, increases length of rope
float kk = 10000;  //TRY-IT: How does changing k affect resting length of the rope? increasing k, shortens rope length
float kv = 1;

// Horizontal params
float kkH = 4000;
float kvH = 1;
float restLenH = .15;
float maxLenH = .3;

float ballRad = 1;

static int maxNodes = 100;
static int maxRopes = 100;
Vec3 pos[][] = new Vec3[maxNodes][maxRopes];
Vec3 vel[][] = new Vec3[maxNodes][maxRopes];
Vec3 acc[][] = new Vec3[maxNodes][maxRopes];

int numNodes = 40;
int numRopes = 50;

Vec3 ballPos[] = new Vec3[n];

void initParams() {
  for (int i = 0; i < n; i++){
    ballPos[i] = new Vec3(0,0,-50);
  }
  for (int h = 0; h < numRopes; h++){
    for (int i = 0; i < numNodes; i++){
      pos[h][i] = new Vec3(0,0,0);
      pos[h][i].x = -5 + stringTop.x + h*.15;
      pos[h][i].y = stringTop.y;
      pos[h][i].z = -50 + stringTop.z - i*.15;
      vel[h][i] = new Vec3(0,0,0);
    }
  }
}

void update(float dt) {
  for (int h = 0; h < numRopes; h++){
    for (int i = 0; i < numNodes; i++){
      acc[h][i] = new Vec3(0,0,0);
      acc[h][i].add(gravity);
    }
  }
  
  // Dampening – Vertical
  for (int h = 0; h < numRopes; h++){
    for (int i = 0; i < numNodes-1; i++){
      Vec3 diff = pos[h][i+1].minus(pos[h][i]);
      float stringF = -kk*(diff.length() - restLen);
      
      Vec3 stringDir = diff.normalized();
      float projVbot = dot(vel[h][i], stringDir);
      float projVtop = dot(vel[h][i+1], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      
      Vec3 force = stringDir.times(stringF + dampF);
      acc[h][i].add(force.times(-1/mass));
      acc[h][i+1].add(force.times(1/mass));
      
      // friction
      acc[h][i].subtract(vel[h][i].times(fric));
    }
  }
  
  // For horizontal
  for (int i = 0; i < numRopes-1; i++){
    for (int j = 0; j < numNodes; j++){
      Vec3 diff = pos[i+1][j].minus(pos[i][j]);
      float stringF = -kkH*(diff.length() - restLenH);
      
      Vec3 dir = diff.normalized();
      float projVbot = dot(vel[i][j], dir);
      float projVtop = dot(vel[i+1][j], dir);
      float dampF = -kvH*(projVtop - projVbot);
      
      Vec3 f = dir.times(stringF + dampF);
      acc[i][j].add(f.times(-1*mass));
      acc[i+1][j].add(f.times(1*mass));

    }
  }
  
  // drag application
  for (int i = 1; i < numRopes-1; i++){
    for (int j = 0; j < numNodes-1; j++){
      Vec3 f1 = dragCalc(pos[i][j],
                         pos[i][j+1],
                         pos[i+1][j],
                         vel[i][j],
                         vel[i][j+1],
                         vel[i+1][j]);
     Vec3 f2 = dragCalc(pos[numRopes-i][numNodes-(j+1)],
                         pos[numRopes-i][numNodes-(j+2)],
                         pos[numRopes-(i+1)][numNodes-(j+1)],
                         vel[numRopes-i][numNodes-(j+1)],
                         vel[numRopes-i][numNodes-(j+2)],
                         vel[numRopes-(i+1)][numNodes-(j+1)]);
      if (!Float.isNaN(f1.x)){
      acc[i][j].add(f1);
      acc[i+1][j].add(f1);
      acc[i][j+1].add(f1);
      acc[i+1][j].add(f2);
      acc[i][j+1].add(f2);
      acc[numRopes-i][numNodes-(j+1)].add(f2);
      }
    }
  }
  // Eulerian integration
  if (euler) {
    for (int h = 0; h < numRopes; h++){
      for(int i = 1; i < numNodes; i++){

        vel[h][i].add(acc[h][i].times(dt));
        pos[h][i].add(vel[h][i].times(dt));
      }
    }
  }
  
  // midpoint method
  if (midpoint) {
    for (int h = 0; h < numRopes; h++){
      for (int i = 1; i < numNodes; i++) {
        
        for (int x = 0; x < 1; x++){
          Vec3 k1 = vel[h][i].plus(acc[h][i].times(dt*.5)); 
          vel[h][i] = k1.plus(acc[h][i].times(dt));
          pos[h][i].add(vel[h][i].times(dt));
        }
      }
    }
  }
  
  // advanced ball collision
  //for (int i = 0; i < numRopes; i++){
  //  for (int j = 1; j < numNodes; j++){
  //   //Step 1: Compute V - a normalized vector pointing from the start of the line segment to the end of the line segment
  //     Vec3 v = pos[i][j].plus(vel[i][j]);
  //     float v_len = v.length(); //Save the distance of the line
  //     v.normalize();
       
  //     hitInfo hitCircle = lineCircleListIntesect(ballPos, rads, pos[i][j], v, v_len);
       
  //     boolean colliding = hitCircle.hit;
  //     float t = hitCircle.t;
  //     float d = pos[i][j].distanceTo(ballPos[0]);

  //     if (colliding ) {
  //      //Vec3 norm = ballPos[0].minus(pos[i][j].times(1));
  //      //norm.normalize();
  //      //Vec3 bounce = norm.times(dot(vel[i][j], norm));
  //      //vel[i][j].subtract(bounce.times(1.0));
        
  //      //pos[i][j].add(norm.times(.001*( ballRad - d)));
  //      //fill(0,0,255);
  //      //Vec3 dir = vel[i][j];
  //      //dir.normalize();
  //      //pushMatrix();
  //      //fill(0,0,255);
  //      //translate(pos[i][j].x + dir.x*t, pos[i][j].y + dir.y*t, pos[i][j].z + dir.z*t );
  //      //sphere(.5);
  //      //popMatrix();
  //   }
  //  }
  //}
  
  
  
  // basic collision detection
  for (int i = 0; i < numRopes; i++){
    for (int j = 0; j < numNodes; j++){
      float d = pos[i][j].distanceTo(ballPos[0]);
      if (d < 0.09 + ballRad*2) {
        Vec3 norm = ballPos[0].minus(pos[i][j].times(1));
        norm.normalize();
        Vec3 bounce = norm.times(dot(vel[i][j], norm));
       
        vel[i][j].subtract(bounce.times(1.75));
        
        pos[i][j].add(norm.times(.001*( ballRad - d)));
      }
    }
  }
  
}

void draw() {
  background(20, 35, 200);
  lights();
  if (reset) initParams();
  cam.Update(1/frameRate);
  if (!paused){
    for (int i = 0; i < numNodes; i++){
      update(1/(20*frameRate));
    }
  }
  // wind settings
  if (windUp) {
    airF.x += .5;
    airF.y += .5;
    airF.z += .5;
  }
  if (windDown) {
    airF.x -= .5;
    airF.y -= .5;
    airF.z -= .5;
  }
  
    // MOVE BALL

    if (forward) {
      ballPos[0].z -= 30/(20*frameRate);
    }
    if (back) {
      ballPos[0].z += 30/(20*frameRate);
    }
    if (left) {
      ballPos[0].x -= 30/(20*frameRate);
    }
    if (right) {
      ballPos[0].x += 30/(20*frameRate);
    }
    if (up) {
      ballPos[0].y += 30/(20*frameRate);
    }
    if (down) {
      ballPos[0].y -= 30/(20*frameRate);
    }   
   
  //for (int h = 0; h < numRopes; h++){
  //  for (int i = 0; i < numNodes-1; i++){
  //    pushMatrix();
  //    pushStyle();
  //    stroke(255, 211, 0);
  //    line(pos[h][i].x, pos[h][i].y, pos[h][i].z, pos[h][i+1].x, pos[h][i+1].y, pos[h][i+1].z);
  //    popStyle();
  //    pushStyle();
  //    noStroke();
  //    translate(pos[h][i+1].x, pos[h][i+1].y, pos[h][i+1].z);
  //    // sphere(radius);
  //    popStyle();
  //    popMatrix();
  //  }
  //}
  
  
  for (int h = 0; h < numRopes-1; h++){
    for (int i = 0; i < numNodes-1; i++){
      pushMatrix();
      pushStyle();
      noStroke();
      //stroke(255, 211, 0);
      //line(pos[h][i].x, pos[h][i].y, pos[h][i].z, pos[h+1][i].x, pos[h+1][i].y, pos[h+1][i].z);
      beginShape();
      //fill((h*i)%random(200), h+1, pow(h,i)%155, random(255));
      fill(h*h*i%255, i*i%255, h*h*h%255);
      vertex(pos[h][i].x, pos[h][i].y, pos[h][i].z);
      vertex(pos[h+1][i].x, pos[h+1][i].y, pos[h+1][i].z);
      vertex(pos[h+1][i+1].x, pos[h+1][i+1].y, pos[h+1][i+1].z);
      vertex(pos[h][i+1].x,pos[h][i+1].y, pos[h][i+1].z);
      endShape();
      
      popStyle();
      pushStyle();
      noStroke();
      translate(pos[h+1][i].x, pos[h+1][i].y, pos[h+1][i].z);
      //sphere(radius);
      popStyle();
      popMatrix();
    }
  }
  
  //fill(150);
  //pushMatrix();
  //translate(0,-10,0);
  //sphere(5);
  //popMatrix();
  
  fill(255);
  rectMode(CENTER);
  pushMatrix();
  rotateX(1.57);
  translate(0,-50,-10);
  rect(0, 0, 100, 100);
  popMatrix();
  for (int i = 0; i < n; i++){
    fill( 0, 0, 255 );
    pushMatrix();
    translate( ballPos[i].x, ballPos[i].y, ballPos[i].z );
    sphere( ballRad*2 );
    popMatrix();
  }
}


// Ball interaction
void keyPressed()
{
  cam.HandleKeyPressed();
    if (key == ' ')
    paused = !paused;
    
   if (key == 'h') {
     windUp = true;
   }
   if (key == 'g') {
     windDown = true;
   }
   
   // ball movements
    if (key == 'k'){
      back = true;
  }
    if (key == 'i'){
      forward = true;
  }
    if (key == 'l'){
      right = true;
  }
    if (key == 'j'){
      left = true;
  }
    if (key == 'u') {
      up = true;
    }
    if (key == 'o') {
      down = true;
    }
    if (key == 'b') {
      reset = true;
      paused = !paused;
    }
}

void keyReleased()
{
      // wind
      if (key == 'h') windUp = false;
      if (key == 'g') windDown = false;  
  
      // ball movements
      if (key == 'i') forward = false;
      if (key == 'k') back = false;
      if (key == 'l') right = false;
      if (key == 'j') left = false;
      if (key == 'u') up = false;
      if (key == 'o') down = false;
      if (key == 'b') reset = false;
      if (key == 'm') {
        midpoint = !midpoint;
        euler = !euler;
      }
  cam.HandleKeyReleased();
}

// Drag calculation function
Vec3 dragCalc(Vec3 a, Vec3 b, Vec3 c, Vec3 va, Vec3 vb, Vec3 vc) {
  
  // first calculate the average velocity and apply air force. 
  Vec3 avgVel = va.plus(vb.plus(vc));
  avgVel = avgVel.divide(3);
  avgVel = avgVel.minus(airF);
  
  // next find normal
  Vec3 ba = b.minus(a);
  Vec3 ca = c.minus(a);
  Vec3 baCrossca = cross(ba, ca);
  float magn = baCrossca.length();
  Vec3 norm = baCrossca.normalized();

  // cross sectional area
  float a0 = .5*magn;
  float aC = dot(avgVel, norm);
  aC = aC/avgVel.length();
  aC *= a0;
  float vSqu = avgVel.length();
  vSqu *= vSqu;
  Vec3 dragForce = norm.times(-.5*airDens*dragCoeff*vSqu*aC);
  dragForce = dragForce.divide(3);

  return dragForce;
}


// collision detection
float strokeWidth = 0;

class hitInfo{
  public boolean hit = false;
  public float t = 5;
}

hitInfo lineCircleIntesect(Vec3 center, float r, Vec3 l_start, Vec3 l_dir, float l_len, float max_t){
  hitInfo hit = new hitInfo();
  
  //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec3 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Lenght of l_dir (we noramlized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - (r+strokeWidth)*(r+strokeWidth); //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the length of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only take the first collision [is this safe?]
      float t2 = (b - sqrt(d))/(2*a);
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < l_len && t1 < max_t){
        hit.hit = true;
        hit.t = t1;
      } 
      else if (t2 > 0 && t2 < l_len && t2 < max_t){
        hit.hit = true;
        hit.t = t2;
      }
    }
    
  return hit;
}

hitInfo lineCircleListIntesect(Vec3[] centers, float[] radii, Vec3 l_start, Vec3 l_dir, float l_len){
  hitInfo hit = new hitInfo();
  hitInfo temp = new hitInfo();
  //TODO: This is only the first circle, loop over all of them
  for (int i = 0; i < n; i++) {
  Vec3 center = centers[i];
  float r = radii[i];
  hitInfo circleHit = lineCircleIntesect(center, r, l_start, l_dir, l_len, hit.t);
  hit.hit = circleHit.hit;
  hit.t = circleHit.t;
  //if (centers[i].distanceTo(l_start) < radii[i]){
  //  fill(255,0,0);
  //  stroke(0,255,250);
  //  return hit;
  //}
    if (hit.hit){
      if (hit.t < temp.t){
        temp.hit = true;
        temp.t = hit.t;
      }
    }
  }
  return temp;
}

Vec3 dxdt(float t, Vec3 x, Vec3 a) {
  return x.plus(a.times(t));
}
